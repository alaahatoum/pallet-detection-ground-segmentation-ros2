from inference_sdk import InferenceHTTPClient
import os
import cv2
import random

# ==== CONFIG ====
API_KEY = "ROBOFLOW_API_KEY"  
MODEL_ID = "pallet_model_v2/1" 
INPUT_DIR = "data/raw"
OUTPUT_DIR = "data/raw"
CLASS_NAME = "pallet"  

NUM_TO_VISUALIZE = 5

# ==== INIT ====
client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
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
        class_id = 0  # Single class: pallet
        x = pred["x"]
        y = pred["y"]
        w = pred["width"]
        h = pred["height"]

        # Convert to YOLO normalized format
        x_center = x / width
        y_center = y / height
        w_norm = w / width
        h_norm = h / height

        yolo_line = f"{class_id} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}"
        yolo_lines.append(yolo_line)

    # Write to .txt file
    base_name = os.path.splitext(filename)[0]
    output_path = os.path.join(OUTPUT_DIR, base_name + ".txt")

    with open(output_path, "w") as f:
        f.write("\n".join(yolo_lines))

    all_results.append((filename, result["predictions"]))
    print(f"Saved annotations for {filename} → {output_path}")

# ==== VISUAL CHECK ====
print(f"\n🔍 Showing {NUM_TO_VISUALIZE} randomly selected results for visual inspection...")
sampled = random.sample(all_results, min(NUM_TO_VISUALIZE, len(all_results)))

for filename, preds in sampled:
    img_path = os.path.join(INPUT_DIR, filename)
    img = cv2.imread(img_path)

    for pred in preds:
        x, y, w, h = int(pred["x"]), int(pred["y"]), int(pred["width"]), int(pred["height"])
        conf = pred["confidence"]
        
        x1, y1 = x - w // 2, y - h // 2
        x2, y2 = x + w // 2, y + h // 2

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{CLASS_NAME} {conf:.2f}"
        cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow(f"Inference: {filename}", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
