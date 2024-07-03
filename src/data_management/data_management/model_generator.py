import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.applications import VGG16, EfficientNetV2L
from tensorflow.keras.layers import Dense, GlobalAveragePooling2D, Dropout
from tensorflow.keras.models import Model, Sequential
from tensorflow.image import flip_left_right, adjust_brightness, adjust_contrast

import os

class ModelGenerator():
    def __init__(self, num_classes, dataset_path, export_path):
        # Set the path to your image dataset folder
        self.dataset_path = dataset_path

        # Set the path to the folder where you want to export the model
        self.export_path = os.path.join(export_path, "model.keras")

        # Set the number of classes in your dataset
        self.num_classes = num_classes

        # Set the input shape of the images
        self.input_shape = (224, 224, 3)

        # Load the pre-trained VGG16 model
        self.base_model = VGG16(weights='imagenet', include_top=False, input_shape=self.input_shape)

        self.base_model_2 = EfficientNetV2L(weights='imagenet', 
                        include_top=False,
                        input_shape= self.input_shape)

    def generate(self):
        # Freeze the base model layers
        for layer in self.base_model.layers:
            layer.trainable = False

        for layer in self.base_model_2.layers:
            layer.trainable = False

        # Create a new model on top of the pre-trained base model
        x = self.base_model.output
        x = GlobalAveragePooling2D()(x)
        x = Dense(1024, activation='relu')(x)
        predictions = Dense(self.num_classes, activation='softmax')(x)
        model = Model(inputs=self.base_model.input, outputs=predictions)

        my_model = Sequential([self.base_model_2,
                       GlobalAveragePooling2D(),
                       Dense(512, activation='relu'),
                       Dropout(0.2),
                       Dense(512, activation='relu'),
                       Dropout(0.2),
                       Dense(self.num_classes, activation='softmax')])
        
        
        BATCH_SIZE = 4
        SEED = 1

        train_dataset = tf.keras.utils.image_dataset_from_directory(
            self.dataset_path, 
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
            self.dataset_path, 
            color_mode='rgb',
            batch_size=BATCH_SIZE,
            image_size=(224, 224),
            shuffle=False,
            validation_split=0.2, #training set is 80% of the data
            subset='validation',
            seed=SEED, 
            label_mode='categorical'
        )

        train_dataset = train_dataset.map(self.augment, num_parallel_calls=tf.data.AUTOTUNE) # call augment everytime image is loaded

        train_dataset = train_dataset.prefetch(buffer_size=tf.data.AUTOTUNE)

        val_dataset = val_dataset.prefetch(buffer_size=tf.data.AUTOTUNE)

        my_model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
        # Compile the model
        model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        # Create an image data generator
        datagen = ImageDataGenerator(rescale=1./255, validation_split=0.2)
        print("going to train_gen")
        
        # Load and preprocess the images from the dataset folder
        train_generator = datagen.flow_from_directory(
            self.dataset_path,
            target_size=self.input_shape[:2],
            batch_size=32,
            class_mode='categorical',
            subset='training'
        )
    
        # Train the model
        model.fit(train_generator, epochs=3)
        # my_model.fit(train_dataset, validation_data=val_dataset, epochs=3)
        my_model.fit(train_dataset, epochs=3, validation_data=val_dataset)
        class_names = list(train_generator.class_indices.keys())
        print("Class Names:", class_names)
        print(model.summary())
        print(my_model.summary())
        # Export the model to the export folder in the proper Keras format
        # model.save(self.export_path)
        my_model.save(self.export_path)

    def augment(self, image, label):
        image = flip_left_right(image)
        image = adjust_brightness(image, delta=0.1)
        image = adjust_contrast(image, contrast_factor=1.75)

        return image, label
