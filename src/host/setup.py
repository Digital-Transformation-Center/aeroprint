from setuptools import setup, find_packages
import os
from glob import glob

package_name = "host"

setup(
    name=package_name,
    version="0.0.0",
    packages=['host', 'host.gui', 'host.gui.gui_pages', 'host.gui.gui_pages.tensor_flow', 'host.gui.resources', 'host.gui.gui_pages.settings', 'host.gui.gui_pages.flight', 'host.web', 'host.web.templates', 'host.web.templates.widgets'],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # Install all HTML templates
        (
            os.path.join("share", package_name, "web", "templates"),
            glob(os.path.join("host", "web", "templates", "*.html")),
        ),
        # Install all HTML widgets
        (
            os.path.join("share", package_name, "web", "templates", "widgets"),
            glob(os.path.join("host", "web", "templates", "widgets", "*.html")),
        ),
        # Install all static files (CSS, JS, etc.)
        # (
        #     os.path.join("share", package_name, "web", "static"),
        #     glob(os.path.join("host", "web", "static", "*")),
        # ),
        (
            os.path.join("share", package_name, "web", "static", "flight_config"),
            glob(os.path.join("host", "web", "static", "flight_config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ryan",
    maintainer_email="ryankuederle@icloud.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # "test_node = host.test_node:main",
            "test_sub = host.test_sub:main",
            "pc_processor = host.pc_processor:main",
            "gui = host.aeroprint_gui:main",
            "pc-collection = host.pc_collection:main", 
            "pc-post-processor = host.pc_post_processor:main", 
            "mesher = host.mesher:main",
            "legacy_gui = host.legacy_aeroprint_gui:main",
            "flask-server-node = host.web.flask_server_node:main",
            "test-node = host.web.test_node:main",
        ],
    },
    package_data={
        'host': [
        'web/templates/*.html',
        'web/templates/widgets/*.html',
        'web/static/flight_config/*',
]
    }
)
