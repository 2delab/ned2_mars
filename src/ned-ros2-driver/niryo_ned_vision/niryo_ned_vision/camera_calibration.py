import cv2
import numpy as np
import time

# === Parameters ===
chessboard_size = (9, 6)
square_size = 25.0
max_frames = 20
CAMERA_INDEX = 0  # Change this if your camera is not index 0 (e.g., 1, 2)

# === Prepare object points ===
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size
objpoints = []
imgpoints = []
img_size = None

# === Init CV2 Capture ===
print("[INFO] Initializing CV2 camera...")
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print(f"[ERROR] Cannot open camera index {CAMERA_INDEX}.")
    exit(-1)

# Wait for camera to stabilize
time.sleep(2)
print("[INFO] Camera ready")

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
cv2.namedWindow("Calibration - CV2", cv2.WINDOW_NORMAL)

print("\n" + "="*60)
print("CAMERA CALIBRATION (CV2)")
print("="*60)
print(f"Target frames: {max_frames}")
print("\nControls:\n  SPACE - Capture frame\n  ESC   - Quit calibration")
print("="*60 + "\n")

last_capture_time = 0
capture_cooldown = 1.0
no_frame_warning_count = 0

try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            no_frame_warning_count += 1
            if no_frame_warning_count > 50:
                print("[WARN] No frames received...")
                no_frame_warning_count = 0
            if cv2.waitKey(1) & 0xFF == 27: break
            continue
            
        no_frame_warning_count = 0
        frame = cv2.flip(frame, -1)  # Flip both horizontally and vertically (180 degrees)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if img_size is None:
            img_size = gray.shape[::-1]
            print(f"[INFO] Image size detected: {img_size[0]}x{img_size[1]}")
            
        found, corners = cv2.findChessboardCorners(
            gray, chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        display = frame.copy()
        current_time = time.time()
        can_capture = (current_time - last_capture_time) > capture_cooldown
        
        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, chessboard_size, corners_refined, found)
            
            progress = f'{len(objpoints)}/{max_frames}'
            if can_capture:
                msg = f'Corners OK - SPACE to capture [{progress}]'
                color = (0, 255, 0)
            else:
                msg = f'Wait {capture_cooldown - (current_time - last_capture_time):.1f}s [{progress}]'
                color = (0, 165, 255)
            cv2.putText(display, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        else:
            cv2.putText(display, 'Position chessboard in view...', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Progress bar
        bar_width = display.shape[1] - 20
        cv2.rectangle(display, (10, display.shape[0] - 30), (10 + bar_width, display.shape[0] - 10), (50, 50, 50), -1)
        progress_width = int((len(objpoints) / max_frames) * bar_width)
        cv2.rectangle(display, (10, display.shape[0] - 30), (10 + progress_width, display.shape[0] - 10), (0, 255, 0), -1)
        
        cv2.imshow("Calibration - CV2", display)
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27: break # ESC
        elif key == 32 and found and can_capture: # SPACE
            objpoints.append(objp.copy())
            imgpoints.append(corners_refined)
            last_capture_time = current_time
            print(f"[INFO] Frame {len(objpoints)}/{max_frames} captured ✓")
            if len(objpoints) >= max_frames:
                print("\n[INFO] ✓ Collected all required frames!")
                break

finally:
    cap.release()
    cv2.destroyAllWindows()

# === Calibration ===
print("\n" + "="*60)
if len(objpoints) >= 5 and img_size is not None:
    print(f"[INFO] Running calibration with {len(objpoints)} frames...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )
    print("\n" + "="*60)
    print("CALIBRATION RESULTS")
    print("="*60)
    print(f"RMS Re-projection Error: {ret:.4f} pixels")
    
    if ret < 1.0: print("  → Excellent calibration!")
    elif ret < 2.0: print("  → Good calibration")
    else: print("  → Fair calibration (consider recapturing)")
        
    print("\nCamera Matrix:")
    print(camera_matrix)
    
    # Save results
    np.savez("camera_calibration.npz",
             camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
             rvecs=rvecs, tvecs=tvecs, img_size=img_size, rms_error=ret)
    print("\n✓ Calibration data saved to: camera_calibration.npz")
    print("="*60 + "\n")
else:
    print("="*60)
    print(f"[ERROR] Not enough valid frames ({len(objpoints)}/5 minimum). Aborted.")
    print("="*60 + "\n")