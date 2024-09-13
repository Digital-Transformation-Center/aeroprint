import tensorflow as tf
import os
from PIL import Image
import numpy as np

def load_model(model_folder):
   model = tf.keras.models.load_model(model_folder)
   return model

# Example usage

path = os.path.dirname(os.path.abspath(__file__))  
# Split the path into components
path_parts = path.split(os.sep)

# Find the index of 'aeroprint' in the path
try:
   aeroprint_index = path_parts.index("aeroprint")
except ValueError:
   print("Error: 'aeroprint' not found in the path")
else:
   # Construct the truncated path
   truncated_path = os.sep.join(path_parts[:aeroprint_index + 1])

model_folder = os.path.join(truncated_path, 'src/data_management/data_management/datasets/thumbs/model/model.keras')  
loaded_model = load_model(model_folder)

test_image_path = os.path.join(truncated_path, 'src/data_management/data_management/datasets/thumbs/images/down/180.jpeg')
image = Image.open(test_image_path)
image = image.resize((224, 224))

# Preprocess the image
image = np.array(image) / 255.0
image = np.expand_dims(image, axis=0)

# Make predictions
predictions = loaded_model.predict(image)

classes = sorted(['down', 'up'])
# Get the predicted class
predicted_class = classes[np.argmax(predictions[0])]

# Print the predicted class
# Get the certainty of the prediction
certainty = np.max(predictions[0])
if certainty < 0.5:
    print("Not Sure")
else:
   print("Predicted class:", predicted_class)
print("Certainty:", certainty)