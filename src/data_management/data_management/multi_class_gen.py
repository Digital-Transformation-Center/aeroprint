import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.applications import VGG16
from tensorflow.keras.layers import Dense, GlobalAveragePooling2D
from tensorflow.keras.models import Model
import os

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

# Set the path to your image dataset folder
dataset_path = os.path.join(truncated_path, 'src/data_management/data_management/datasets/self_images') 

# Set the path to the folder where you want to export the model
export_path = os.path.join(truncated_path, 'src/data_management/data_management/models/thumbs.keras') 

# Set the number of classes in your dataset
num_classes = 2

# Set the input shape of the images
input_shape = (224, 224, 3)

# Load the pre-trained VGG16 model
base_model = VGG16(weights='imagenet', include_top=False, input_shape=input_shape)

# Freeze the base model layers
for layer in base_model.layers:
    layer.trainable = False

# Create a new model on top of the pre-trained base model
x = base_model.output
x = GlobalAveragePooling2D()(x)
x = Dense(1024, activation='relu')(x)
predictions = Dense(num_classes, activation='softmax')(x)
model = Model(inputs=base_model.input, outputs=predictions)

# Compile the model
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

# Create an image data generator
datagen = ImageDataGenerator(rescale=1./255, validation_split=0.2)

# Load and preprocess the images from the dataset folder
train_generator = datagen.flow_from_directory(
    dataset_path,
    target_size=input_shape[:2],
    batch_size=32,
    class_mode='categorical',
    subset='training'
)

# Train the model
model.fit(train_generator, epochs=3)
class_names = list(train_generator.class_indices.keys())
print("Class Names:", class_names)
print(model.summary())
# Export the model to the export folder in the proper Keras format
model.save(export_path)