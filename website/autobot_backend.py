# """
# AutoBot Dashboard Backend
# - Hosts the HTML dashboard
# - MQTT bridge for ROS2 topics (/imu/data, /cmd_vel)
# - OpenCV color detection via Logitech C270
# - Remote execution of ROS2 bags and sudo pkill
# """
# import os
# import cv2
# import json
# import time
# import base64
# import threading
# import subprocess
# import numpy as np
# from datetime import datetime
# from flask import Flask, jsonify, request, send_from_directory
# from flask_cors import CORS
# import paho.mqtt.client as mqtt

# # --- Configuration ---
# MQTT_BROKER = "127.0.0.1"       # <-- Change to Raspberry Pi IP if running remotely
# MQTT_PORT   = 1883
# MQTT_TOPICS = {
#     "imu":     "/imu/data",
#     "cmd_vel": "/cmd_vel",
#     "eta":     "/autobot/eta",
#     "stop":    "/autobot/stop",
# }
# CAMERA_INDEX   = 0              # Logitech C270 index
# SUCCESS_FOLDER = "images_success"
# FLASK_PORT     = 5050

# # --- State Dictionary ---
# state = {
#     "imu": {
#         "or_x": 0.0, "or_y": 0.0, "or_z": 0.0, "or_w": 0.0,
#         "ang_x": 0.0, "ang_y": 0.0, "ang_z": 0.0,
#         "lin_x": 0.0, "lin_y": 0.0, "lin_z": 0.0,
#     },
#     "cmd_vel": {
#         "linear_x": 0.0, "linear_y": 0.0,
#         "angular_z": 0.0,
#     },
#     "eta": {"seconds": 0, "method": "N/A"},
#     "last_detection": None,
#     "mqtt_connected": False,
# }

# os.makedirs(SUCCESS_FOLDER, exist_ok=True)

# # --- MQTT Setup ---
# def on_connect(client, userdata, flags, rc):
#     if rc == 0:
#         state["mqtt_connected"] = True
#         print("[MQTT] Connected to broker")
#         for topic in MQTT_TOPICS.values():
#             client.subscribe(topic)
#     else:
#         print(f"[MQTT] Connection failed: {rc}")

# def on_message(client, userdata, msg):
#     try:
#         payload = json.loads(msg.payload.decode())
#         topic   = msg.topic

#         if topic == MQTT_TOPICS["imu"]:
#             orientation = payload.get("orientation", {})
#             angular = payload.get("angular_velocity", {})
#             linear = payload.get("linear_acceleration", {})
#             state["imu"].update({
#                 "or_x": round(orientation.get("x", 0), 3),
#                 "or_y": round(orientation.get("y", 0), 3),
#                 "or_z": round(orientation.get("z", 0), 3),
#                 "or_w": round(orientation.get("w", 0), 3),
#                 "ang_x": round(angular.get("x", 0), 3),
#                 "ang_y": round(angular.get("y", 0), 3),
#                 "ang_z": round(angular.get("z", 0), 3),
#                 "lin_x": round(linear.get("x", 0), 3),
#                 "lin_y": round(linear.get("y", 0), 3),
#                 "lin_z": round(linear.get("z", 0), 3),
#             })
#         elif topic == MQTT_TOPICS["cmd_vel"]:
#             lin = payload.get("linear", {})
#             ang = payload.get("angular", {})
#             state["cmd_vel"].update({
#                 "linear_x":  round(lin.get("x", 0), 3),
#                 "linear_y":  round(lin.get("y", 0), 3),
#                 "angular_z": round(ang.get("z", 0), 3),
#             })
#         elif topic == MQTT_TOPICS["eta"]:
#             state["eta"].update({
#                 "seconds": payload.get("seconds", 0),
#                 "method":  payload.get("method", "N/A"),
#             })
#     except Exception as e:
#         print(f"[MQTT] Parse error: {e}")

# mqtt_client = mqtt.Client()
# mqtt_client.on_connect = on_connect
# mqtt_client.on_message = on_message

# def start_mqtt():
#     try:
#         mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
#         mqtt_client.loop_forever()
#     except Exception as e:
#         print(f"[MQTT] Could not connect: {e}")

# # --- Color Detection ---
# def detect_color(image):
#     h, w = image.shape[:2]
#     x1, y1 = w // 4, h // 4
#     x2, y2 = 3 * w // 4, 3 * h // 4
#     roi = image[y1:y2, x1:x2]

#     hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#     mean_v = np.mean(hsv[:, :, 2])

#     is_black = mean_v < 50
#     color    = "black" if is_black else "not-black"
#     rack     = "Rack 1" if is_black else "Rack 4"

#     return {
#         "color": color, "rack": rack, "mean_brightness": round(float(mean_v), 2),
#         "bbox": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
#         "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
#     }

# def capture_and_detect():
#     cap = cv2.VideoCapture(CAMERA_INDEX)
#     if not cap.isOpened():
#         return {"error": "Camera not available"}

#     ret, frame = cap.read()
#     cap.release()

#     if not ret:
#         return {"error": "Failed to capture frame"}

#     result = detect_color(frame)
#     b = result["bbox"]
#     annotated = frame.copy()
#     cv2.rectangle(annotated, (b["x1"], b["y1"]), (b["x2"], b["y2"]), (0, 255, 0), 2)
#     label = f"{result['color'].upper()} -> {result['rack']}"
#     cv2.putText(annotated, label, (b["x1"], b["y1"] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#     fname = f"{SUCCESS_FOLDER}/capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
#     cv2.imwrite(fname, annotated)
#     result["saved_as"] = fname

#     _, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
#     result["image_b64"] = base64.b64encode(buf).decode("utf-8")
#     state["last_detection"] = result
#     return result

# # --- Flask API ---
# app = Flask(__name__, static_folder=".")
# CORS(app)

# @app.route("/api/status")
# def api_status():
#     return jsonify({
#         "imu":            state["imu"],
#         "cmd_vel":        state["cmd_vel"],
#         "eta":            state["eta"],
#         "mqtt_connected": state["mqtt_connected"],
#         "last_detection": {k: v for k, v in (state["last_detection"] or {}).items() if k != "image_b64"},
#     })

# @app.route("/api/capture", methods=["POST"])
# def api_capture():
#     return jsonify(capture_and_detect())

# # # @app.route("/api/play_bag", methods=["POST"])
# # # def api_play_bag():
# # #     """Triggered by Mac, executes rosbag on the Pi."""
# # #     data = request.json
# # #     task = data.get("task")
    
# # #     bag_paths = {
# # #         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
# # #         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
# # #     }
    
# # #     target_bag = bag_paths.get(task)
# # #     if not target_bag:
# # #         return jsonify({"error": "Invalid task selected"}), 400

# # #     try:
# # #         # Kill currently playing bags to avoid overlapping data
# # #         subprocess.run(["sudo", "pkill", "-f", "ros2 bag play"], check=False)
# # #         time.sleep(0.5) 
        
# # #         # Launch new bag in background, source Jazzy, and remap the topic!
# # #         command = f"source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel_corrected:=/cmd_vel"
# # #         subprocess.Popen(command, shell=True, executable='/bin/bash')
        
# # #         print(f"[BAG] Started playing {target_bag} on Pi with Jazzy environment")
# # #         return jsonify({"status": f"Task {task} bag started on Pi!"})
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500

# # @app.route("/api/play_bag", methods=["POST"])
# # def api_play_bag():
# #     data = request.json
# #     task = data.get("task")
# #     bag_paths = {
# #         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
# #         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
# #     }
# #     target_bag = bag_paths.get(task)
# #     if not target_bag: return jsonify({"error": "Invalid task"}), 400

# #     try:
# #         # Kill any currently running bags to prevent overlapping
# #         os.system("pkill -f 'ros2 bag play'")
# #         time.sleep(1) 
        
# #         # The bulletproof command using 'nohup' and '&' 
# #         # This completely detaches the process from Flask so it doesn't crash
# #         command = f"source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel_corrected:=/cmd_vel"
# #         os.system(f"nohup /bin/bash -c '{command}' > /dev/null 2>&1 &")
        
# #         return jsonify({"status": f"Task {task} bag started on Pi!"})
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500

# # @app.route("/api/play_bag", methods=["POST"])
# # def api_play_bag():
# #     data = request.json
# #     task = data.get("task")
# #     bag_paths = {
# #         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
# #         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
# #     }
# #     target_bag = bag_paths.get(task)
# #     if not target_bag: return jsonify({"error": "Invalid task"}), 400

# #     try:
# #         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
# #         time.sleep(0.5) 
        
# #         # MAGIC FIX: We added `source ~/.bashrc` and we are logging all output to bag_error_log.txt
# #         command = f"source ~/.bashrc && source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel_corrected:=/cmd_vel > /home/rbcnvamsi/Desktop/bag_error_log.txt 2>&1"
        
# #         subprocess.Popen(command, shell=True, executable='/bin/bash')
# #         return jsonify({"status": f"Task {task} triggered. Checking for errors..."})
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500

# # @app.route("/api/play_bag", methods=["POST"])
# # def api_play_bag():
# #     data = request.json
# #     task = data.get("task")
# #     bag_paths = {
# #         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
# #         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
# #     }
# #     target_bag = bag_paths.get(task)
# #     if not target_bag: return jsonify({"error": "Invalid task"}), 400

# #     try:
# #         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
# #         time.sleep(0.5) 
        
# #         # THE FIX: Added --disable-keyboard-controls so it runs smoothly in the background
# #         command = f"source ~/.bashrc && source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel_corrected:=/cmd_vel --disa............................ble-keyboard-controls"
        
# #         subprocess.Popen(command, shell=True, executable='/bin/bash')
# #         return jsonify({"status": f"Task {task} bag started successfully!"})
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500

# @app.route("/api/play_bag", methods=["POST"])
# def api_play_bag():
#     data = request.json
#     task = data.get("task")
#     bag_paths = {
#         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
#         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
#     }
#     target_bag = bag_paths.get(task)
#     if not target_bag: return jsonify({"error": "Invalid task"}), 400

#     try:
#         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
#         time.sleep(0.5) 
        
#         # THE FIX: Remapping from /cmd_vel (in the bag) to /cmd_vel_corrected (for your robot base)
#         command = f"source ~/.bashrc && source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel:=/cmd_vel_corrected --disable-keyboard-controls"
        
#         subprocess.Popen(command, shell=True, executable='/bin/bash')
#         return jsonify({"status": f"Task {task} bag started successfully!"})
#     except Exception as e:
#         return jsonify({"error": str(e)}), 500

# @app.route("/api/stop", methods=["POST"])
# def api_stop():
#     try:
#         # 1. Instantly kill the ROS 2 bag player (so it stops sending old commands)
#         # Note: We use "ros2 bag play" instead of "ros" so we don't accidentally kill your motor drivers or bridge!
#         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
        
#         # 2. Slam the brakes! Send a single 0-velocity command to the wheels
#         brake_cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel_corrected geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
#         subprocess.Popen(brake_cmd, shell=True, executable='/bin/bash')
        
#         return jsonify({"status": "🚨 EMERGENCY STOP EXECUTED!"})
#     except Exception as e:
#         return jsonify({"error": str(e)}), 500

# @app.route("/api/last_image")
# def api_last_image():
#     d = state.get("last_detection")
#     if d and "image_b64" in d:
#         return jsonify({"image_b64": d["image_b64"], "result": {k: v for k, v in d.items() if k != "image_b64"}})
#     return jsonify({"error": "No image captured yet"})

# @app.route("/")
# def serve_dashboard():
#     # Ensure this matches the exact name of your HTML file!
#     # If your file is named autobot_dashboard(1).html, update this string.
#     return send_from_directory(".", "autobot_dashboard.html")

# # --- Entry Point ---
# if __name__ == "__main__":
#     print("=" * 50)
#     print("  AutoBot Dashboard Backend")
#     print(f"  MQTT Broker : {MQTT_BROKER}:{MQTT_PORT}")
# #     print(f"  Dashboard   : http://0.0.0.0:{FLASK_PORT}")
# #     print("=" * 50)

# #     mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
# #     mqtt_thread.start()
# #     app.run(host="0.0.0.0", port=FLASK_PORT, debug=False)


# # # """
# # # AutoBot Dashboard Backend
# # # - Run this ON THE RASPBERRY PI!
# # # """
# # # import os
# # # import cv2
# # # import json
# # # import time
# # # import base64
# # # import threading
# # # import subprocess
# # # import numpy as np
# # # from datetime import datetime
# # # from flask import Flask, jsonify, request, send_from_directory
# # # from flask_cors import CORS
# # # import paho.mqtt.client as mqtt

# # # # --- Configuration ---
# # # MQTT_BROKER = "127.0.0.1"       # Works perfectly since backend is on the Pi
# # # MQTT_PORT   = 1883
# # # MQTT_TOPICS = {
# # #     "imu":     "/imu/data",
# # #     "cmd_vel": "/cmd_vel",
# # #     "eta":     "/autobot/eta",
# # #     "stop":    "/autobot/stop",
# # # }
# # # CAMERA_INDEX   = 0              # Logitech C270 index on the Pi
# # # SUCCESS_FOLDER = "images_success"
# # # FLASK_PORT     = 5050

# # # # --- State Dictionary ---
# # # state = {
# # #     "imu": {"or_x": 0.0, "or_y": 0.0, "or_z": 0.0, "or_w": 0.0, "ang_x": 0.0, "ang_y": 0.0, "ang_z": 0.0, "lin_x": 0.0, "lin_y": 0.0, "lin_z": 0.0},
# # #     "cmd_vel": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
# # #     "eta": {"seconds": 0, "method": "N/A"},
# # #     "last_detection": None,
# # #     "mqtt_connected": False,
# # # }

# # # os.makedirs(SUCCESS_FOLDER, exist_ok=True)

# # # # --- MQTT Setup ---
# # # def on_connect(client, userdata, flags, rc):
# # #     if rc == 0:
# # #         state["mqtt_connected"] = True
# # #         print("[MQTT] Connected to broker on localhost")
# # #         for topic in MQTT_TOPICS.values():
# # #             client.subscribe(topic)

# # # def on_message(client, userdata, msg):
# # #     try:
# # #         payload = json.loads(msg.payload.decode())
# # #         topic   = msg.topic
# # #         if topic == MQTT_TOPICS["imu"]:
# # #             orientation = payload.get("orientation", {})
# # #             angular = payload.get("angular_velocity", {})
# # #             linear = payload.get("linear_acceleration", {})
# # #             state["imu"].update({
# # #                 "or_x": round(orientation.get("x", 0), 3), "or_y": round(orientation.get("y", 0), 3), "or_z": round(orientation.get("z", 0), 3),
# # #                 "ang_z": round(angular.get("z", 0), 3), "lin_x": round(linear.get("x", 0), 3), "lin_y": round(linear.get("y", 0), 3),
# # #             })
# # #         elif topic == MQTT_TOPICS["cmd_vel"]:
# # #             lin = payload.get("linear", {})
# # #             ang = payload.get("angular", {})
# # #             state["cmd_vel"].update({
# # #                 "linear_x": round(lin.get("x", 0), 3), "linear_y": round(lin.get("y", 0), 3), "angular_z": round(ang.get("z", 0), 3),
# # #             })
# # #     except Exception as e:
# # #         pass

# # # mqtt_client = mqtt.Client()
# # # mqtt_client.on_connect = on_connect
# # # mqtt_client.on_message = on_message

# # # def start_mqtt():
# # #     mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
# # #     mqtt_client.loop_forever()

# # # # --- Color Detection ---
# # # def detect_color(image):
# # #     h, w = image.shape[:2]
# # #     x1, y1, x2, y2 = w // 4, h // 4, 3 * w // 4, 3 * h // 4
# # #     roi = image[y1:y2, x1:x2]
# # #     hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
# # #     mean_v = np.mean(hsv[:, :, 2])
# # #     is_black = mean_v < 50
# # #     return {
# # #         "color": "black" if is_black else "not-black", "rack": "Rack 1" if is_black else "Rack 4",
# # #         "mean_brightness": round(float(mean_v), 2), "bbox": {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
# # #     }

# # # def capture_and_detect():
# # #     cap = cv2.VideoCapture(CAMERA_INDEX)
# # #     if not cap.isOpened(): return {"error": "Camera not available on Pi"}
# # #     ret, frame = cap.read()
# # #     cap.release()
# # #     if not ret: return {"error": "Failed to capture frame"}

# # #     result = detect_color(frame)
# # #     b = result["bbox"]
# # #     cv2.rectangle(frame, (b["x1"], b["y1"]), (b["x2"], b["y2"]), (0, 255, 0), 2)
# # #     _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
# # #     result["image_b64"] = base64.b64encode(buf).decode("utf-8")
# # #     state["last_detection"] = result
# # #     return result

# # # # --- Flask API ---
# # # app = Flask(__name__, static_folder=".")
# # # CORS(app)

# # # @app.route("/api/status")
# # # def api_status():
# # #     return jsonify({
# # #         "imu": state["imu"], "cmd_vel": state["cmd_vel"], "eta": state["eta"],
# # #         "mqtt_connected": state["mqtt_connected"]
# # #     })

# # # @app.route("/api/capture", methods=["POST"])
# # # def api_capture():
# # #     return jsonify(capture_and_detect())

# # # @app.route("/api/play_bag", methods=["POST"])
# # # def api_play_bag():
# # #     data = request.json
# # #     task = data.get("task")
# # #     bag_paths = {
# # #         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
# # #         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
# # #     }
# # #     target_bag = bag_paths.get(task)
# # #     if not target_bag: return jsonify({"error": "Invalid task"}), 400

# # #     try:
# # #         # Removed 'sudo' to prevent script from hanging for password
# # #         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
# # #         time.sleep(0.5) 
# # #         command = f"source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel_corrected:=/cmd_vel"
# # #         subprocess.Popen(command, shell=True, executable='/bin/bash')
# # #         return jsonify({"status": f"Task {task} bag started on Pi!"})
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route("/api/stop", methods=["POST"])
# # # def api_stop():
# # #     try:
# # #         # Removed 'sudo' here as well
# # #         subprocess.run(["pkill", "-f", "ros"], check=False)
# # #         return jsonify({"status": "ROS processes killed!"})
# # #     except Exception as e:
# # #         return jsonify({"error": "Failed to kill process"}), 500

# # # @app.route("/")
# # # def serve_dashboard():
# # #     # Matches your newly renamed file!
# # #     return send_from_directory(".", "autobot.html")

# # # if __name__ == "__main__":
# # #     threading.Thread(target=start_mqtt, daemon=True).start()
# # #     app.run(host="0.0.0.0", port=FLASK_PORT, debug=False)

# """
# AutoBot Dashboard Backend
# - Hosts the HTML dashboard
# - MQTT bridge for ROS2 topics (/imu/data, /cmd_vel)
# - Remote execution of ROS2 bags and sudo pkill
# * NOTE: Hardware Camera (cv2) removed. Detection is now handled Client-Side in browser.
# """
# import os
# import json
# import time
# import threading
# import subprocess
# from flask import Flask, jsonify, request, send_from_directory
# from flask_cors import CORS
# import paho.mqtt.client as mqtt

# # --- Configuration ---
# MQTT_BROKER = "127.0.0.1"       
# MQTT_PORT   = 1883
# MQTT_TOPICS = {
#     "imu":     "/imu/data",
#     "cmd_vel": "/cmd_vel",
#     "eta":     "/autobot/eta",
#     "stop":    "/autobot/stop",
# }
# FLASK_PORT     = 5050

# # --- State Dictionary ---
# state = {
#     "imu": {
#         "or_x": 0.0, "or_y": 0.0, "or_z": 0.0, "or_w": 0.0,
#         "ang_x": 0.0, "ang_y": 0.0, "ang_z": 0.0,
#         "lin_x": 0.0, "lin_y": 0.0, "lin_z": 0.0,
#     },
#     "cmd_vel": {
#         "linear_x": 0.0, "linear_y": 0.0,
#         "angular_z": 0.0,
#     },
#     "eta": {"seconds": 0, "method": "N/A"},
#     "mqtt_connected": False,
# }

# # --- MQTT Setup ---
# def on_connect(client, userdata, flags, rc):
#     if rc == 0:
#         state["mqtt_connected"] = True
#         print("[MQTT] Connected to broker")
#         for topic in MQTT_TOPICS.values():
#             client.subscribe(topic)
#     else:
#         print(f"[MQTT] Connection failed: {rc}")

# def on_message(client, userdata, msg):
#     try:
#         payload = json.loads(msg.payload.decode())
#         topic   = msg.topic

#         if topic == MQTT_TOPICS["imu"]:
#             orientation = payload.get("orientation", {})
#             angular = payload.get("angular_velocity", {})
#             linear = payload.get("linear_acceleration", {})
#             state["imu"].update({
#                 "or_x": round(orientation.get("x", 0), 3),
#                 "or_y": round(orientation.get("y", 0), 3),
#                 "or_z": round(orientation.get("z", 0), 3),
#                 "or_w": round(orientation.get("w", 0), 3),
#                 "ang_x": round(angular.get("x", 0), 3),
#                 "ang_y": round(angular.get("y", 0), 3),
#                 "ang_z": round(angular.get("z", 0), 3),
#                 "lin_x": round(linear.get("x", 0), 3),
#                 "lin_y": round(linear.get("y", 0), 3),
#                 "lin_z": round(linear.get("z", 0), 3),
#             })
#         elif topic == MQTT_TOPICS["cmd_vel"]:
#             lin = payload.get("linear", {})
#             ang = payload.get("angular", {})
#             state["cmd_vel"].update({
#                 "linear_x":  round(lin.get("x", 0), 3),
#                 "linear_y":  round(lin.get("y", 0), 3),
#                 "angular_z": round(ang.get("z", 0), 3),
#             })
#         elif topic == MQTT_TOPICS["eta"]:
#             state["eta"].update({
#                 "seconds": payload.get("seconds", 0),
#                 "method":  payload.get("method", "N/A"),
#             })
#     except Exception as e:
#         pass

# mqtt_client = mqtt.Client()
# mqtt_client.on_connect = on_connect
# mqtt_client.on_message = on_message

# def start_mqtt():
#     try:
#         mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
#         mqtt_client.loop_forever()
#     except Exception as e:
#         print(f"[MQTT] Could not connect: {e}")

# # --- Flask API ---
# app = Flask(__name__, static_folder=".")
# CORS(app)

# @app.route("/api/status")
# def api_status():
#     return jsonify({
#         "imu":            state["imu"],
#         "cmd_vel":        state["cmd_vel"],
#         "eta":            state["eta"],
#         "mqtt_connected": state["mqtt_connected"],
#     })

# @app.route("/api/play_bag", methods=["POST"])
# def api_play_bag():
#     data = request.json
#     task = data.get("task")
#     bag_paths = {
#         1: "/home/rbcnvamsi/Desktop/pbl/pbl_task_2_path",
#         2: "/home/rbcnvamsi/Desktop/pbl/pbl_task_6_path"
#     }
#     target_bag = bag_paths.get(task)
#     if not target_bag: return jsonify({"error": "Invalid task"}), 400

#     try:
#         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
#         time.sleep(0.5) 
#         command = f"source ~/.bashrc && source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel:=/cmd_vel_corrected --disable-keyboard-controls"
#         subprocess.Popen(command, shell=True, executable='/bin/bash')
#         return jsonify({"status": f"Task {task} bag started successfully!"})
#     except Exception as e:
#         return jsonify({"error": str(e)}), 500

# @app.route("/api/stop", methods=["POST"])
# def api_stop():
#     try:
#         subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
#         brake_cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel_corrected geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
#         subprocess.Popen(brake_cmd, shell=True, executable='/bin/bash')
#         return jsonify({"status": "🚨 EMERGENCY STOP EXECUTED!"})
#     except Exception as e:
#         return jsonify({"error": str(e)}), 500

# @app.route("/")
# def serve_dashboard():
#     return send_from_directory(".", "autobot_dashboard.html")

# # --- Entry Point ---
# if __name__ == "__main__":
#     print("=" * 50)
#     print("  AutoBot Dashboard Backend (Pi-Optimized)")
#     print(f"  MQTT Broker : {MQTT_BROKER}:{MQTT_PORT}")
#     print(f"  Dashboard   : http://0.0.0.0:{FLASK_PORT}")
#     print("=" * 50)

#     mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
#     mqtt_thread.start()
#     app.run(host="0.0.0.0", port=FLASK_PORT, debug=False)



"""
AutoBot Dashboard Backend
- Hosts the HTML dashboard over HTTPS (Required for Laptop Camera)
- Subscribes to MQTT topics (/imu/data, /cmd_vel, /odom)
"""
import os
import json
import time
import threading
import subprocess
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
import paho.mqtt.client as mqtt

MQTT_BROKER = "127.0.0.1"       
MQTT_PORT   = 1883
MQTT_TOPICS = {
    "imu":     "/imu/data",
    "cmd_vel": "/cmd_vel",
    "odom":    "/odom",
    "eta":     "/autobot/eta",
    "stop":    "/autobot/stop",
}
FLASK_PORT = 5050

state = {
    "imu": { "or_x": 0.0, "or_y": 0.0, "or_z": 0.0, "or_w": 0.0, "ang_z": 0.0, "lin_x": 0.0 },
    "cmd_vel": { "linear_x": 0.0, "angular_z": 0.0 },
    "odom": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "eta": {"seconds": 0, "method": "N/A"},
    "mqtt_connected": False,
}

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        state["mqtt_connected"] = True
        print("[MQTT] Connected to broker")
        for topic in MQTT_TOPICS.values():
            client.subscribe(topic)

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        topic = msg.topic

        if topic == MQTT_TOPICS["imu"]:
            o = payload.get("orientation", {})
            a = payload.get("angular_velocity", {})
            state["imu"].update({
                "or_x": round(o.get("x", 0), 3), "or_y": round(o.get("y", 0), 3),
                "or_z": round(o.get("z", 0), 3), "or_w": round(o.get("w", 0), 3),
                "ang_z": round(a.get("z", 0), 3)
            })
        elif topic == MQTT_TOPICS["cmd_vel"]:
            state["cmd_vel"].update({
                "linear_x":  round(payload.get("linear", {}).get("x", 0), 3),
                "angular_z": round(payload.get("angular", {}).get("z", 0), 3),
            })
        elif topic == MQTT_TOPICS["odom"]:
            pose = payload.get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            state["odom"]["position"].update({
                "x": pos.get("x", 0.0), "y": pos.get("y", 0.0), "z": pos.get("z", 0.0)
            })
            state["odom"]["orientation"].update({
                "x": ori.get("x", 0.0), "y": ori.get("y", 0.0), "z": ori.get("z", 0.0), "w": ori.get("w", 1.0)
            })
    except Exception as e:
        pass

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

def start_mqtt():
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_forever()
    except Exception:
        print("[MQTT] Connection error")

app = Flask(__name__, static_folder=".")
CORS(app)

@app.route("/api/status")
def api_status():
    return jsonify(state)

@app.route("/api/play_bag", methods=["POST"])
def api_play_bag():
    task = request.json.get("task")
    bag_paths = {
        1: "/home/rbcnvamsi/Desktop/pbl/task_21_path",
        2: "/home/rbcnvamsi/Desktop/pbl/task_24_path"
    }
    target_bag = bag_paths.get(task)
    if not target_bag: return jsonify({"error": "Invalid task"}), 400

    try:
        subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
        time.sleep(0.5) 
        command = f"source ~/.bashrc && source /opt/ros/jazzy/setup.bash && ros2 bag play {target_bag} --remap /cmd_vel:=/cmd_vel --disable-keyboard-controls"
        subprocess.Popen(command, shell=True, executable='/bin/bash')
        return jsonify({"status": f"Task {task} bag started!"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/api/stop", methods=["POST"])
def api_stop():
    try:
        subprocess.run(["pkill", "-f", "ros2 bag play"], check=False)
        brake_cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /cmd_vel_corrected geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        subprocess.Popen(brake_cmd, shell=True, executable='/bin/bash')
        return jsonify({"status": "🚨 STOPPED!"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/")
def serve_dashboard():
    return send_from_directory(".", "autobot_dashboard.html")

if __name__ == "__main__":
    mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
    mqtt_thread.start()
    print("=" * 50)
    print("  AutoBot Backend Active")
    print(f"  Access via: https://<Raspberry_Pi_IP>:{FLASK_PORT}")
    print("  * YOU MUST USE HTTPS FOR CAMERA TO WORK *")
    print("=" * 50)
    # ssl_context='adhoc' forces HTTPS, solving the camera permission block
    app.run(host="0.0.0.0", port=FLASK_PORT, ssl_context='adhoc', debug=False)