from roboflow import Roboflow

# Initialize Roboflow with your API key
rf = Roboflow(api_key="qGpYhxLCtK9qm6NSVpra")

# Connect to your project
project = rf.workspace("robot-potk5").project("industry_dataset3")
version = project.version(1)

# Download the dataset in YOLOv8 format
dataset = version.download("yolov8")
