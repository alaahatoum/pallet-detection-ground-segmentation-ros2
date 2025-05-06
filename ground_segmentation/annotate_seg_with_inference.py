from inference_sdk import InferenceHTTPClient
from dotenv import load_dotenv
import os
import cv2
import random
import numpy as np

load_dotenv()

# ==== CONFIG ====
API_KEY = os.getenv("ROBOFLOW_API_KEY")
MODEL_ID = "ground_segmentation-hgwoo/2"  
INPUT_DIR = "ground_segmentation/data"
OUTPUT_DIR = "ground_segmentation/labels"
CLASS_NAME = "ground"  
CLASS_ID = 0          

NUM_TO_VISUALIZE = 5
def inference_annotaion():
    # ==== INIT ====
    client = InferenceHTTPClient(
        api_url="https://serverless.roboflow.com",  # ensure this is updated
        api_key=API_KEY
    )

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    all_results = []

    # ==== MAIN LOOP ====
    for filename in os.listdir(INPUT_DIR):
        if not filename.lower().endswith((".jpg", ".jpeg", ".png")):
            continue

        image_path = os.path.join(INPUT_DIR, filename)
        image = cv2.imread(image_path)
        height, width = image.shape[:2]

        result = client.infer(image_path, model_id=MODEL_ID)

        yolo_lines = []

        for pred in result["predictions"]:
            if "points" not in pred:
                continue  # skip non-segmentation results

            points = pred["points"]
            if len(points) < 3:
                continue  # skip invalid polygons

            yolo_line = f"{CLASS_ID}"
            for pt in points:
                x_norm = pt["x"] / width
                y_norm = pt["y"] / height
                yolo_line += f" {x_norm:.6f} {y_norm:.6f}"
            yolo_lines.append(yolo_line)

        # Write to .txt file
        base_name = os.path.splitext(filename)[0]
        output_path = os.path.join(OUTPUT_DIR, base_name + ".txt")

        with open(output_path, "w") as f:
            f.write("\n".join(yolo_lines))

        all_results.append((filename, image, result["predictions"]))
        print(f"Saved segmentation for {filename} → {output_path}")

# ==== VISUAL CHECK ====
all_results = []

for filename in os.listdir(INPUT_DIR):
    if not filename.lower().endswith((".jpg", ".jpeg", ".png")):
        continue

    base_name = os.path.splitext(filename)[0]
    txt_path = os.path.join(OUTPUT_DIR, base_name + ".txt")
    if not os.path.exists(txt_path):
        continue

    image_path = os.path.join(INPUT_DIR, filename)
    image = cv2.imread(image_path)
    height, width = image.shape[:2]

    preds = []
    with open(txt_path, "r") as f:
        for line in f.readlines():
            parts = line.strip().split()
            if len(parts) < 7:  # At least 3 points (x,y) + class
                continue

            class_id = int(parts[0])
            coords = list(map(float, parts[1:]))
            points = []
            for i in range(0, len(coords), 2):
                x = coords[i] * width
                y = coords[i + 1] * height
                points.append({"x": x, "y": y})

            preds.append({"points": points, "confidence": 1.0})  # Fake confidence

    all_results.append((filename, image, preds))

print(f"\n🔍 Showing {NUM_TO_VISUALIZE} randomly selected results for visual inspection...")
sampled = random.sample(all_results, min(NUM_TO_VISUALIZE, len(all_results)))

for filename, img, preds in sampled:
    for pred in preds:
        if "points" not in pred:
            continue
        pts = [(int(pt["x"]), int(pt["y"])) for pt in pred["points"]]
        if len(pts) >= 3:
            cv2.polylines(img, [np.array(pts, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
            conf = pred.get("confidence", 0)
            label = f"{CLASS_NAME} {conf:.2f}"
            cv2.putText(img, label, pts[0], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow(f"Segmentation: {filename}", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
