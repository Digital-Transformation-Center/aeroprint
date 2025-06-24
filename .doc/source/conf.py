# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

project = 'AeroPrint'
copyright = '2024, Digital Transformation Center'
author = 'Ryan Kuederle, Tim Marshall'
release = '1.0.1'

sys.path.insert(0, os.path.abspath('../../src/host'))
sys.path.insert(0, os.path.abspath('../../src/starling'))

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx_rtd_dark_mode'
]

default_dark_mode = True

autodoc_mock_imports = [
    'pyqt5',
    'pyyaml',
    'numpy',
    'px4_msgs',
    'rclpy',
    'geometry_msgs',
    'px4_msgs',
    'open3d',
    'cv2',
    'tensorflow',
    'PIL',
    'trimesh',
    'std_msgs',
    'PyQt5',
    'sensor_msgs'

]

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = [
    'custom.css',
]

