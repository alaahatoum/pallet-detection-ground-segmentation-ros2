from roboflow import Roboflow
from dotenv import load_dotenv
import os

load_dotenv()
# Initialize Roboflow with your API key
rf = Roboflow(api_key = os.getenv("ROBOFLOW_API_KEY"))

# Connect to your project
project = rf.workspace("robot-potk5").project("industry_dataset3")
version = project.version(1)

# Download the dataset in YOLOv8 format
dataset = version.download("yolov8")

# configuration allows for customization of the dataset download
