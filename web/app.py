# app.py
import os
import cv2
import uuid
import uvicorn
import numpy as np
from fastapi import FastAPI, Request, File, UploadFile, Form, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from ultralytics import YOLO
import json
import base64
from datetime import datetime
from fastapi.middleware.cors import CORSMiddleware
import asyncio
from typing import List

os.makedirs("static", exist_ok=True)
os.makedirs("static/css", exist_ok=True)
os.makedirs("static/js", exist_ok=True)
os.makedirs("static/results", exist_ok=True)

app = FastAPI(title="Weed and Crop Detection System")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory="templates")

model = YOLO('best.pt')

class_names = ['crop', 'weed']

detection_history = []
HISTORY_LIMIT = 10

class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            await connection.send_json(message)

manager = ConnectionManager()

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request, "latest_detection": detection_history[-1] if detection_history else None})

@app.get("/history_page", response_class=HTMLResponse)
async def history_page(request: Request):
    return templates.TemplateResponse("history.html", {"request": request, "history": detection_history})

@app.get("/capture_page", response_class=HTMLResponse)
async def capture_page(request: Request):
    return templates.TemplateResponse("capture_page.html", {"request": request})

@app.get("/api/history")
async def get_history():
    return {"status": "success", "history": detection_history}

@app.get("/api/latest_detection_summary")
async def get_latest_detection_summary():
    if detection_history:
        latest_detection = detection_history[-1]
        return {"status": "success", "latest_detection": latest_detection}
    else:
        return {"status": "success", "latest_detection": None, "message": "No detections yet."}

@app.get("/api/latest_detection_image")
async def get_latest_detection_image():
    if detection_history:
        latest_detection = detection_history[-1]
        try:
            with open(latest_detection["image_url"], "rb") as image_file:
                encoded_image = base64.b64encode(image_file.read()).decode('utf-8')
            return JSONResponse({
                "status": "success",
                "image": f"data:image/jpeg;base64,{encoded_image}"
            })
        except Exception as e:
            return JSONResponse({"status": "error", "message": f"Error loading latest image: {str(e)}"})
    else:
        return JSONResponse({"status": "success", "image": None, "message": "No detections yet."})

@app.get("/api/live_feed_image")
async def get_live_feed_image():
    try:
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            cap.release()
            return JSONResponse({"status": "error", "message": "Failed to open camera"})
        
        ret, frame = cap.read()
        
        cap.release()
        
        if not ret:
            return JSONResponse({"status": "error", "message": "Failed to capture image from camera"})
        
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            return JSONResponse({"status": "error", "message": "Failed to encode image"})
        
        img_str = base64.b64encode(buffer).decode('utf-8')
        
        return JSONResponse({
            "status": "success",
            "image": f"data:image/jpeg;base64,{img_str}"
        })
    
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)})

@app.post("/api/detect_image/")
async def detect_image(image_data: str = Form(...), confidence: float = Form(0.7)):
    try:
        frame_bytes = base64.b64decode(image_data.split(",")[1])
        np_arr = np.frombuffer(frame_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = model(frame, conf=confidence)[0]

        detections = []
        for r in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = r
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            detections.append({
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "confidence": float(conf),
                "class": int(cls),
                "class_name": class_names[int(cls)],
                "center_pixel": [center_x, center_y]
            })

        counts = {}
        for det in detections:
            class_name = det["class_name"]
            if class_name in counts:
                counts[class_name] += 1
            else:
                counts[class_name] = 1

        for det in detections:
            x1, y1, x2, y2 = map(int, det["bbox"])
            cls = det["class"]
            color = (0, 255, 0) if cls == 0 else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{det['class_name']} {det['confidence']:.2f}",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        _, buffer = cv2.imencode('.jpg', frame)
        img_str = base64.b64encode(buffer).decode('utf-8')

        if detections:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            result_id = str(uuid.uuid4())
            result_filename = f"static/results/{result_id}.jpg"
            cv2.imwrite(result_filename, frame)

            history_item = {
                "id": result_id,
                "timestamp": timestamp,
                "detections": detections,
                "counts": counts,
                "image_url": result_filename
            }
            detection_history.append(history_item)
            detection_history[:] = detection_history[-HISTORY_LIMIT:]

        return JSONResponse({
            "status": "success",
            "image": f"data:image/jpeg;base64,{img_str}",
            "detections": detections,
            "counts": counts
        })

    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)})

@app.websocket("/ws/camera_feed")
async def websocket_camera_feed(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            try:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    await websocket.send_json({
                        "status": "error", 
                        "message": "Failed to open camera"
                    })
                    cap.release()
                    await asyncio.sleep(1)
                    continue
                
                ret, frame = cap.read()
                cap.release()
                
                if not ret:
                    await websocket.send_json({
                        "status": "error", 
                        "message": "Failed to capture image from camera"
                    })
                    await asyncio.sleep(1)
                    continue
                
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    await websocket.send_json({
                        "status": "error", 
                        "message": "Failed to encode image"
                    })
                    await asyncio.sleep(1)
                    continue
                
                img_str = base64.b64encode(buffer).decode('utf-8')
                
                await websocket.send_json({
                    "status": "success",
                    "image": f"data:image/jpeg;base64,{img_str}"
                })
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                await websocket.send_json({
                    "status": "error",
                    "message": str(e)
                })
                await asyncio.sleep(1)
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.on_event("startup")
async def startup_event():
    print("Starting Weed and Crop Detection System...")
    print(f"Model loaded: YOLOv8")
    print(f"Classes: {class_names}")

if __name__ == "__main__":
    uvicorn.run("app:app", host="0.0.0.0", port=8000, reload=True)