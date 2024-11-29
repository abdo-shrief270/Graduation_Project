import cv2
import face_recognition
import os
import numpy as np

# Path to known faces directory
known_faces_dir = r"C:\Users\20155\PycharmProjects\FaceRecognition\known_faces"
tolerance = 0.4  # Lowered for higher accuracy
model = "hog"  # Switch to cnn for better accuracy if supported

# Load known faces
known_face_encodings = []
known_face_names = []

for filename in os.listdir(known_faces_dir):
    if filename.endswith(".jpg") or filename.endswith(".png"):
        image = face_recognition.load_image_file(os.path.join(known_faces_dir, filename))
        encoding = face_recognition.face_encodings(image)[0]
        known_face_encodings.append(encoding)
        known_face_names.append(os.path.splitext(filename)[0])  # Name from filename

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Resize frame for better quality
    small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    # Detect faces and encode them
    face_locations = face_recognition.face_locations(rgb_small_frame, model=model)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    for face_encoding, face_location in zip(face_encodings, face_locations):
        # Check if the face matches any known faces
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=tolerance)
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)

        name = "Unknown"
        if matches[best_match_index]:
            name = known_face_names[best_match_index]

        # Adjust rectangle size
        top, right, bottom, left = face_location
        top *= 2
        right *= 2
        bottom *= 2
        left *= 2
        padding = 20  # Increase the rectangle size by adding padding
        cv2.rectangle(frame, (left - padding, top - padding), (right + padding, bottom + padding), (0, 255, 0), 2)

        # Display name and accuracy
        accuracy = round((1 - face_distances[best_match_index]) * 100, 2)
        label = f"{name} ({accuracy}%)"
        cv2.putText(frame, label, (left - padding, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Face Recognition", frame)

    # Break on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()

