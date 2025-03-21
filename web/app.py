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

# Create necessary directories
os.makedirs("static", exist_ok=True)
os.makedirs("static/css", exist_ok=True)
os.makedirs("static/js", exist_ok=True)
os.makedirs("static/results", exist_ok=True)

app = FastAPI(title="Weed and Crop Detection System")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount static files directory
app.mount("/static", StaticFiles(directory="static"), name="static")

# Set up Jinja2 templates
templates = Jinja2Templates(directory="templates")

# Load YOLOv8 model - replace 'best.pt' with the path to your model
model = YOLO('best.pt')

# Class names for the model (replace with your actual class names)
class_names = ['crop', 'weed']

# Store detection history (limit to 10 recent detections)
detection_history = []
HISTORY_LIMIT = 10

# Add a connection manager class
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

# Create a connection manager
manager = ConnectionManager()

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request, "latest_detection": detection_history[-1] if detection_history else None}) # Pass latest detection for homepage

@app.get("/history_page", response_class=HTMLResponse)
async def history_page(request: Request):
    """Route to display the detection history page."""
    return templates.TemplateResponse("history.html", {"request": request, "history": detection_history}) # Pass history here

@app.get("/capture_page", response_class=HTMLResponse)
async def capture_page(request: Request):
    """Route to display the capture and detect page."""
    return templates.TemplateResponse("capture_page.html", {"request": request})

@app.get("/api/history")
async def get_history():
    """Get detection history"""
    return {"status": "success", "history": detection_history}

@app.get("/api/latest_detection_summary")
async def get_latest_detection_summary():
    """Get summary of the latest detection."""
    if detection_history:
        latest_detection = detection_history[-1]
        return {"status": "success", "latest_detection": latest_detection}
    else:
        return {"status": "success", "latest_detection": None, "message": "No detections yet."}

@app.get("/api/latest_detection_image")
async def get_latest_detection_image():
    """Get the latest detected image."""
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
    """API endpoint to get the current camera feed as a base64-encoded image"""
    try:
        # Initialize camera (you may want to use a camera singleton or global object in production)
        cap = cv2.VideoCapture(0)  # 0 is usually the default camera
        
        # Check if camera opened successfully
        if not cap.isOpened():
            cap.release()
            return JSONResponse({"status": "error", "message": "Failed to open camera"})
        
        # Read a frame
        ret, frame = cap.read()
        
        # Release the camera
        cap.release()
        
        if not ret:
            return JSONResponse({"status": "error", "message": "Failed to capture image from camera"})
        
        # Convert to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            return JSONResponse({"status": "error", "message": "Failed to encode image"})
        
        # Convert to base64
        img_str = base64.b64encode(buffer).decode('utf-8')
        
        # Return as JSON with base64-encoded image
        return JSONResponse({
            "status": "success",
            "image": f"data:image/jpeg;base64,{img_str}"
        })
    
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)})

@app.post("/api/detect_image/")
async def detect_image(image_data: str = Form(...), confidence: float = Form(0.7)):
    """Endpoint to process a single image for weed and crop detection."""
    try:
        frame_bytes = base64.b64decode(image_data.split(",")[1])
        np_arr = np.frombuffer(frame_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Run detection
        results = model(frame, conf=confidence)[0]

        # Process results
        detections = []
        for r in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = r
            center_x = int((x1 + x2) / 2) # Calculate center x
            center_y = int((y1 + y2) / 2) # Calculate center y
            detections.append({
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "confidence": float(conf),
                "class": int(cls),
                "class_name": class_names[int(cls)],
                "center_pixel": [center_x, center_y] # Add center coordinates
            })

        # Count detections by class
        counts = {}
        for det in detections:
            class_name = det["class_name"]
            if class_name in counts:
                counts[class_name] += 1
            else:
                counts[class_name] = 1

        # Generate result image
        for det in detections:
            x1, y1, x2, y2 = map(int, det["bbox"])
            cls = det["class"]
            color = (0, 255, 0) if cls == 0 else (0, 0, 255)  # Green for crops, Red for weeds
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{det['class_name']} {det['confidence']:.2f}",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Convert the frame back to base64
        _, buffer = cv2.imencode('.jpg', frame)
        img_str = base64.b64encode(buffer).decode('utf-8')

        # Save to history if there are detections
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
            # Keep only the last 10 detections
            detection_history[:] = detection_history[-HISTORY_LIMIT:] # Keep last 10

        return JSONResponse({
            "status": "success",
            "image": f"data:image/jpeg;base64,{img_str}",
            "detections": detections,
            "counts": counts
        })

    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)})

# Add a WebSocket endpoint for the camera feed
@app.websocket("/ws/camera_feed")
async def websocket_camera_feed(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        # Start the camera feed broadcast for this connection
        while True:
            try:
                # Capture image from camera
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    await websocket.send_json({
                        "status": "error", 
                        "message": "Failed to open camera"
                    })
                    cap.release()
                    await asyncio.sleep(1)  # Wait before retrying
                    continue
                
                ret, frame = cap.read()
                cap.release()
                
                if not ret:
                    await websocket.send_json({
                        "status": "error", 
                        "message": "Failed to capture image from camera"
                    })
                    await asyncio.sleep(1)  # Wait before retrying
                    continue
                
                # Convert to JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    await websocket.send_json({
                        "status": "error", 
                        "message": "Failed to encode image"
                    })
                    await asyncio.sleep(1)
                    continue
                
                # Convert to base64
                img_str = base64.b64encode(buffer).decode('utf-8')
                
                # Send the image to the client
                await websocket.send_json({
                    "status": "success",
                    "image": f"data:image/jpeg;base64,{img_str}"
                })
                
                # Wait a short time before sending the next frame
                # Adjust this value to control frame rate (0.1 = ~10 FPS)
                await asyncio.sleep(0.1)
                
            except Exception as e:
                await websocket.send_json({
                    "status": "error",
                    "message": str(e)
                })
                await asyncio.sleep(1)  # Wait before retrying
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.on_event("startup")
async def startup_event():
    """Initialize the application"""
    print("Starting Weed and Crop Detection System...")
    print(f"Model loaded: YOLOv8")
    print(f"Classes: {class_names}")

if __name__ == "__main__":
    uvicorn.run("app:app", host="0.0.0.0", port=8000, reload=True)