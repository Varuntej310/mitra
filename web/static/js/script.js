// static/js/script.js (COMPLETE - Corrected to include all functions)

// Function to update the "Latest Detection" section on the homepage
async function updateLatestDetection() {
    try {
        const summaryResponse = await fetch('/api/latest_detection_summary');
        const summaryData = await summaryResponse.json();

        const imageResponse = await fetch('/api/latest_detection_image');
        const imageData = await imageResponse.json();

        if (summaryData.status === "success" && imageData.status === "success") {
            const latestDetection = summaryData.latest_detection;
            const latestImageBase64 = imageData.image;

            if (latestDetection) {
                document.getElementById('timestamp').textContent = latestDetection.timestamp;
                document.getElementById('crop-count').textContent = latestDetection.counts.crop || 0;
                document.getElementById('weed-count').textContent = latestDetection.counts.weed || 0;
                document.getElementById('latest-detection-image').src = latestImageBase64 || ""; // Set image src if available
                document.getElementById('no-detections-message').style.display = 'none'; // Hide "No detections" message
                document.getElementById('latest-detection-image').style.display = 'block'; // Show image
            } else {
                document.getElementById('timestamp').textContent = "N/A";
                document.getElementById('crop-count').textContent = "0";
                document.getElementById('weed-count').textContent = "0";
                document.getElementById('latest-detection-image').src = ""; // Clear image src
                document.getElementById('latest-detection-image').style.display = 'none'; // Hide image
                document.getElementById('no-detections-message').style.display = 'block'; // Show "No detections" message
            }
        } else {
            console.error("Error fetching latest detection data:", summaryData, imageData);
            // Handle error display on webpage if needed
        }

    } catch (error) {
        console.error("Error updating latest detection:", error);
        // Handle fetch errors, maybe display a message on the page
    }
}

// WebSocket connection for camera feed
let cameraSocket = null;

// Function to connect to the camera WebSocket
function connectCameraWebSocket(elementId, errorElementId) {
    // Close any existing connection
    if (cameraSocket) {
        cameraSocket.close();
    }
    
    // Create a new WebSocket connection
    cameraSocket = new WebSocket(`ws://${window.location.host}/ws/camera_feed`);
    
    // Handle connection open
    cameraSocket.onopen = function(e) {
        console.log("Camera WebSocket connection established");
    };
    
    // Handle messages from the server
    cameraSocket.onmessage = function(event) {
        const data = JSON.parse(event.data);
        
        if (data.status === "success") {
            const imageElement = document.getElementById(elementId);
            const errorElement = document.getElementById(errorElementId);
            
            if (imageElement && errorElement) {
                imageElement.src = data.image;
                errorElement.style.display = 'none';
                imageElement.style.display = 'block';
            }
        } else {
            console.error("Error from camera feed:", data.message || "Unknown error");
            const imageElement = document.getElementById(elementId);
            const errorElement = document.getElementById(errorElementId);
            
            if (imageElement && errorElement) {
                imageElement.style.display = 'none';
                errorElement.textContent = "Error: Could not load camera feed. " + (data.message || "Camera not responding");
                errorElement.style.display = 'block';
            }
        }
    };
    
    // Handle errors
    cameraSocket.onerror = function(error) {
        console.error("WebSocket Error:", error);
        const errorElement = document.getElementById(errorElementId);
        if (errorElement) {
            errorElement.textContent = "Error: Could not connect to camera feed. Network error.";
            errorElement.style.display = 'block';
        }
    };
    
    // Handle disconnection
    cameraSocket.onclose = function(event) {
        console.log("Camera WebSocket connection closed");
        // Try to reconnect after a delay
        setTimeout(function() {
            connectCameraWebSocket(elementId, errorElementId);
        }, 2000);
    };
}

// Replace the old updateCaptureLiveFeed function
function setupCapturePage() {
    // Connect to the camera WebSocket for the capture page
    connectCameraWebSocket('capture-camera-feed', 'capture-camera-error-message');
    
    // Set up the capture button event
    document.getElementById('capture-button').addEventListener('click', captureAndDetect);
}

// Function to clean up WebSocket connections when leaving a page
function cleanupWebSockets() {
    if (cameraSocket) {
        cameraSocket.close();
        cameraSocket = null;
    }
}

// Add event to clean up when the page is unloaded
window.addEventListener('beforeunload', cleanupWebSockets);

// Function to capture an image and perform detection
async function captureAndDetect() {
    try {
        // Get the current camera image from the img element
        const currentImage = document.getElementById('capture-camera-feed').src;
        
        // Check if we have a valid image
        if (!currentImage || currentImage === '') {
            throw new Error("No camera image available to capture");
        }
        
        // Show loading state
        document.getElementById('capture-button').disabled = true;
        document.getElementById('capture-button').textContent = 'Processing...';
        
        // Send image for detection
        const formData = new FormData();
        formData.append('image_data', currentImage);
        formData.append('confidence', '0.5'); // Default confidence threshold
        
        const response = await fetch('/api/detect_image/', {
            method: 'POST',
            body: formData
        });
        
        const result = await response.json();
        
        if (result.status === "success") {
            // Display detection results
            document.getElementById('detection-result-container').style.display = 'block';
            document.getElementById('detected-image').src = result.image;
            document.getElementById('capture-timestamp').textContent = result.timestamp;
            document.getElementById('capture-crop-count').textContent = result.counts.crop || 0;
            document.getElementById('capture-weed-count').textContent = result.counts.weed || 0;
            document.getElementById('detection-error-message').style.display = 'none';
        } else {
            // Show error
            document.getElementById('detection-result-container').style.display = 'none';
            document.getElementById('detection-error-message').textContent = 'Error: ' + (result.message || 'Detection failed');
            document.getElementById('detection-error-message').style.display = 'block';
        }
    } catch (error) {
        console.error("Error in capture and detect:", error);
        document.getElementById('detection-result-container').style.display = 'none';
        document.getElementById('detection-error-message').textContent = 'Error: ' + error.message;
        document.getElementById('detection-error-message').style.display = 'block';
    } finally {
        // Reset button state
        document.getElementById('capture-button').disabled = false;
        document.getElementById('capture-button').textContent = 'Take Picture and Detect';
    }
}