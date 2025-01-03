import tensorflow as tf
import os
from PIL import Image
import numpy as np

class ModelTester():
    def __init__(self, model_folder):
      self.model = self.load_model(model_folder)

    def load_model(self, model_folder):
        model = tf.keras.models.load_model(model_folder)
        return model

    def test(self, image):
        # image = Image.open(test_image_path)
        # image = image.resize((224, 224))

        # # Preprocess the image
        # image = np.array(image) / 255.0
        # image = np.expand_dims(image, axis=0)

        # Make predictions
        prediction = self.model.predict(image)

        certainty = np.max(prediction[0])

        return prediction, certainty
