from dotenv import load_dotenv
import os

load_dotenv()  # This looks for a .env file in the current working directory

api_key = os.getenv("ROBOFLOW_API_KEY")
print("API Key loaded:", api_key)
