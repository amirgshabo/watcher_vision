yolo_detector:
  ros__parameters:
    # Model settings
    model_path: "yolov8n.pt"
    
    # Detection thresholds
    confidence_threshold: 0.45
    nms_threshold: 0.50
    
    # Input topic — switch between live ZED and rosbag
    input_topic: "/zed/zed_node/rgb/image_rect_color"
    
    # Depth topic from ZED
    depth_topic: "/zed/zed_node/depth/depth_registered"
    
    # Output topics
    detections_topic: "/yolo_detections"
    annotated_topic: "/yolo_annotated"
    combined_topic: "/yolo_detections_with_depth"
    
    # Filtering — only detect these COCO class IDs
    # 0: person, 2: car, 9: traffic light, 11: stop sign
    target_classes: [0, 2, 9, 11]
    
    # Depth filtering
    depth_filter_kernel_size: 5
    max_depth_range: 10.0