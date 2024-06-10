import json
import os
import argparse
import time
import cv2
import shutil
from sklearn.model_selection import train_test_split

coco_data = {
    "info": {
        "description": "Converted Dataset",
        "version": "0.1",
        "year": time.localtime().tm_year
    },
    "images": [],
    "annotations": [],
    "categories": []
}

image_id = 0
annotation_id = 1
category_id_map = {}

def get_image_dimensions(image_path):
    image = cv2.imread(image_path)
    height, width, _ = image.shape
    return width, height

def add_category(name):
    if name not in category_id_map:
        new_id = len(coco_data["categories"]) + 1
        coco_data["categories"].append({
            "id": new_id,
            "name": name,
            "supercategory": "none"
        })
        category_id_map[name] = new_id
    return category_id_map[name]

def process_captures_file(captures_file, height, width):
    global image_id, annotation_id
    with open(captures_file, 'r') as file:
        captures = json.load(file)['captures']

        for capture in captures:
            image_id += 1  
            image = {
                "id": image_id,
                "file_name": capture['filename'],
                "width": width,  
                "height": height   
            }
            coco_data['images'].append(image)

            for annotation in capture['annotations']:
                for value in annotation['values']:
                    category_name = value.get('category', 'default')  # Use 'default' if 'category' is missing
                    category_id = add_category(category_name)
                    coco_annotation = {
                        "id": annotation_id,
                        "image_id": image_id,
                        "category_id": category_id,  
                        "bbox": [value['x'], value['y'], value['width'], value['height']],
                        "area": value['width'] * value['height'],
                        "segmentation": [],
                        "iscrowd": 0
                    }
                    coco_data['annotations'].append(coco_annotation)
                    annotation_id += 1

def convert_coco_to_yolo(coco_file, output_dir):
    with open(coco_file, 'r') as file:
        data = json.load(file)
    
    images_info = {image['id']: {'file_name': image['file_name'], 'width': image['width'], 'height': image['height']}
                   for image in data['images']}
    
    annotations = {}
    for annotation in data['annotations']:
        image_id = annotation['image_id']
        if image_id not in annotations:
            annotations[image_id] = []
        bbox = annotation['bbox']
        x_center = (bbox[0] + bbox[2] / 2) / images_info[image_id]['width']
        y_center = (bbox[1] + bbox[3] / 2) / images_info[image_id]['height']
        width = bbox[2] / images_info[image_id]['width']
        height = bbox[3] / images_info[image_id]['height']
        annotations[image_id].append((annotation['category_id'], x_center, y_center, width, height))
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    for image_id, bboxes in annotations.items():
        file_name = os.path.splitext(os.path.basename(images_info[image_id]['file_name']))[0] + '.txt'
        shutil.copy(images_info[image_id]['file_name'], output_dir)
        
        file_path = os.path.join(output_dir, file_name)
        with open(file_path, 'w') as file:
            for bbox in bboxes:
                line = ' '.join(map(str, bbox))
                file.write(line + '\n')

def split_data(input_dir, output_dir, test_size=0.2, val_size=0.1):
    images = [f for f in os.listdir(input_dir) if f.endswith('.jpg') or f.endswith('.png')]
    train_val_images, test_images = train_test_split(images, test_size=test_size, random_state=42)
    train_images, val_images = train_test_split(train_val_images, test_size=val_size / (1 - test_size), random_state=42)
    
    train_dir = os.path.join(output_dir, 'train')
    val_dir = os.path.join(output_dir, 'val')
    test_dir = os.path.join(output_dir, 'test')
    
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)
    
    for image in train_images:
        shutil.copy(os.path.join(input_dir, image), train_dir)
    
    for image in val_images:
        shutil.copy(os.path.join(input_dir, image), val_dir)
    
    for image in test_images:
        shutil.copy(os.path.join(input_dir, image), test_dir)
    

def main():
    parser = argparse.ArgumentParser(description='Convert dataset to COCO format and optionally to YOLO format.')
    parser.add_argument('config_path', type=str, help='Path to the config JSON file.')
    parser.add_argument('--convert_to_yolo', action='store_true', help='Convert COCO dataset to YOLO format.')
    parser.add_argument('--output_dir', type=str, default='dataset', help='Output directory for YOLO format.')
    parser.add_argument('--split_data', action='store_true', help='Split data into training, validation, and testing sets.')
    parser.add_argument('--test_size', type=float, default=0.2, help='Proportion of the dataset to include in the test split.')
    parser.add_argument('--val_size', type=float, default=0.1, help='Proportion of the dataset to include in the validation split.')
    args = parser.parse_args()

    config = json.load(open(args.config_path))

    image_path = config['captures'][0]['filename']
    height, width = get_image_dimensions(image_path)

    process_captures_file(args.config_path, height, width)

    coco_output_path = 'coco_formatted.json'
    with open(coco_output_path, 'w') as out_file:
        json.dump(coco_data, out_file, indent=4)

    if args.split_data:
        input_dir = os.path.dirname(image_path)
        split_data(input_dir, args.output_dir, args.test_size, args.val_size)

        if args.convert_to_yolo:
            convert_coco_to_yolo(coco_output_path, os.path.join(args.output_dir, 'train'))
            convert_coco_to_yolo(coco_output_path, os.path.join(args.output_dir, 'val'))
            convert_coco_to_yolo(coco_output_path, os.path.join(args.output_dir, 'test'))
    else:
        if args.convert_to_yolo:
            convert_coco_to_yolo(coco_output_path, args.output_dir)

if __name__ == "__main__":
    main()
