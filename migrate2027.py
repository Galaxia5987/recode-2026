import json
from pathlib import Path

def apply_kotlin_replacements(json_file_path: str, target_dir: str = "./src"):
    # Load replacement configurations from the JSON file
    with open(json_file_path, 'r', encoding='utf-8') as f:
        config = json.load(f)

    replacements = config[0].get("replacements", [])

    # Recursively find all Kotlin files in the directory
    kotlin_files = Path(target_dir).rglob("*.kt")

    for file_path in kotlin_files:
        try:
            content = file_path.read_text(encoding='utf-8')
            modified_content = content

            # Apply each replacement rule sequentially
            for rule in replacements:
                modified_content = modified_content.replace(rule["from"], rule["to"])

            # Overwrite the file only if changes occurred
            if content != modified_content:
                file_path.write_text(modified_content, encoding='utf-8')
                print(f"Modified: {file_path}")

        except Exception as e:
            print(f"Error processing {file_path}: {e}")

if __name__ == "__main__":
    apply_kotlin_replacements("replacements.json")