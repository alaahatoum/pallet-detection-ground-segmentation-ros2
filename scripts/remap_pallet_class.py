import os

# Original class ID for 'pallet' from Roboflow dataset
PALLET_CLASS_ID = "1"
NEW_CLASS_ID = "0"

# Label directories to process
label_dirs = [
    "industry_dataset3-1/train/labels",
    "industry_dataset3-1/valid/labels",
    "industry_dataset3-1/test/labels"
]

def remap_labels(directory):
    if not os.path.exists(directory):
        print(f"[!] Directory not found: {directory}")
        return

    for filename in os.listdir(directory):
        if not filename.endswith(".txt"):
            continue

        filepath = os.path.join(directory, filename)
        new_lines = []

        with open(filepath, "r") as file:
            for line in file:
                parts = line.strip().split()
                if parts and parts[0] == PALLET_CLASS_ID:
                    parts[0] = NEW_CLASS_ID
                    new_lines.append(" ".join(parts))

        with open(filepath, "w") as file:
            file.write("\n".join(new_lines) + ("\n" if new_lines else ""))

        print(f"[✓] Processed: {filepath}")

if __name__ == "__main__":
    for dir_path in label_dirs:
        remap_labels(dir_path)
