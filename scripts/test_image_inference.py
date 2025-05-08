import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

def main():
    # Load models
    package_share_dir = get_package_share_directory('pallet_ground_package')
    models_dir = os.path.join(package_share_dir, 'models')
    ground_model = YOLO(os.path.join(models_dir, 'ground_seg.pt'))
    pallet_model = YOLO(os.path.join(models_dir, 'pallet_det.pt'))

    # Load test image
    image_path = "/home/alaa/Desktop/pallet-detection-ground-segmentation-ros2/ground_segmentation/raw/data/640401-9657_jpg.rf.e368f35da8da3392bf49a4b89173662d.jpg"
    print(f"Loading image from: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print("Failed to load image.")
    else:
        print("Image loaded successfully.")

    # Run models
    ground_result = ground_model(image, verbose=False)[0]
    if ground_result.masks:
        for mask_tensor in ground_result.masks.data:
            mask = mask_tensor.cpu().numpy().astype("uint8") * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

    pallet_result = pallet_model(image, verbose=False)[0]
    if pallet_result.boxes:
        for box in pallet_result.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(image, "Pallet", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Show results
    cv2.imshow("Test Image Inference", image)
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
