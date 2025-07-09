import json


def load_and_process_path(file_path):
    # Load the JSON file
    with open(file_path, "r") as file:
        data = json.load(file)

    # Negate the 'y' value for all key points in all paths
    for path in data.get("paths", []):
        for key_point in path.get("key_points", []):
            key_point["y"] = 8.052 - key_point["y"]
            key_point["y_velocity"] = -key_point["y_velocity"]
            key_point["y_acceleration"] = -key_point["y_acceleration"]
            key_point["angle"] = -key_point["angle"]
            key_point["angular_velocity"] = -key_point["angular_velocity"]
            key_point["angular_acceleration"] = -key_point["angular_acceleration"]

    return data


def main():
    # Example file path
    input_file = "src/main/deploy/3PieceFeederSmart.polarauto"  # Replace with your actual file path
    output_file = "src/main/deploy/Processor_3PieceFeederSmart.polarauto"

    # Process the path
    processed_data = load_and_process_path(input_file)

    # Save the processed data to a new file
    with open(output_file, "w") as file:
        json.dump(processed_data, file, indent=4)

    print(f"Processed path saved to {output_file}")


if __name__ == "__main__":
    main()
