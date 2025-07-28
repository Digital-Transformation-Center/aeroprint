from tensorflow.keras.applications import EfficientNetV2L
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, GlobalAveragePooling2D, Dense, Dropout

model = EfficientNetV2L(weights='imagenet', 
                        include_top=False,
                        input_shape=(224, 224, 3))

for layer in model.layers:
    layer.trainable = False

my_model = Sequential([model,
                       GlobalAveragePooling2D(),
                       Dense(512, activation='relu'),
                       Dropout(0.2),
                       Dense(512, activation='relu'),
                       Dropout(0.2),
                       Dense(3, activation='softmax')])
# For more than two classifications, change the last Dense layer to the number of classes and the activation to 'softmax'

import tensorflow as tf
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
images_path = os.path.join(truncated_path, 'src/data_management/data_management/datasets/images')  

BATCH_SIZE = 2
SEED = 1
train_dataset = tf.keras.utils.image_dataset_from_directory(
   images_path, 
   color_mode='rgb',
   batch_size=BATCH_SIZE,
   image_size=(224, 224),
   shuffle=True,
   validation_split=0.2, #training set is 80% of the data
   subset='training',
   seed=SEED,
   label_mode='categorical'
)


val_dataset = tf.keras.utils.image_dataset_from_directory(
   images_path, 
   color_mode='rgb',
   batch_size=BATCH_SIZE,
   image_size=(224, 224),
   shuffle=False,
   validation_split=0.2, #training set is 80% of the data
   subset='validation',
   seed=SEED, 
   label_mode='categorical'
)


from tensorflow.image import flip_left_right, adjust_brightness, adjust_contrast

def augment(image, label):
   image = flip_left_right(image)
   image = adjust_brightness(image, delta=0.1)
   image = adjust_contrast(image, contrast_factor=1.75)

   return image, label

train_dataset = train_dataset.map(augment, num_parallel_calls=tf.data.AUTOTUNE) # call augment everytime image is loaded

train_dataset = train_dataset.prefetch(buffer_size=tf.data.AUTOTUNE)

val_dataset = val_dataset.prefetch(buffer_size=tf.data.AUTOTUNE)

from tensorflow.keras.metrics import *

metrics = ['accuracy', Precision(), Recall(), AUC()] #Look up the docs on the metrics to make sure something that makes sense is being used

from tensorflow.keras.optimizers import Adam
from tensorflow.keras.losses import BinaryCrossentropy

my_model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

from tensorflow.keras.callbacks import EarlyStopping

es = EarlyStopping(monitor='val_loss', patience=3) #Stop training if val_loss does not improve for 3 epochs - avoids overfitting
my_model.fit(train_dataset, validation_data=val_dataset, epochs=5) # , callbacks=[es])

model_export_dir = os.path.join(truncated_path, 'src/data_management/data_management/models/cat_dog')
print(my_model.summary())
my_model.export(model_export_dir)

