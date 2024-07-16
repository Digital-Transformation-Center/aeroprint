import sqlite3
import pandas as pd

# Test connection with 
try:
    db_connection = sqlite3.connect('qualityDatabase.db')
    print("Database qualityDatabase.db found.")
except:
    print("Database qualityDatabase.db not found. New database formed.")

try:
    db_connection.execute("SELECT name IF EXISTS flights")
except:
    db_connection.execute("""
                CREATE TABLE IF NOT EXISTS flights (
                    id INTEGER PRIMARY KEY,
                    grading_final_grade TEXT,
                    grading_metadata_flight_grade REAL,
                    grading_metadata_process_grade REAL,
                    grading_metadata_scan_grade REAL,
                    grading_collected_automatic_grade REAL,
                    metadata_flight_date TEXT,
                    metadata_flight_duration TEXT,
                    metadata_flight_parameters TEXT,
                    metadata_flight_rosLogs TEXT,
                    metadata_process_flight_success INTEGER,
                    metadata_process_pointcloud_success INTEGER,
                    metadata_process_mesh_success INTEGER,
                    metadata_process_gcode_success INTEGER,
                    metadata_process_imageRecognition_success INTEGER,
                    metadata_process_processingTime_success INTEGER,
                    metadata_scan_closestDistance REAL,
                    metadata_scan_furthestDistance REAL,
                    metadata_scan_averageDistance REAL,
                    metadata_scan_lowestQuality REAL,
                    metadata_scan_highestQuality REAL,
                    metadata_scan_averageQuality REAL,
                    metadata_scan_lowestPPC INTEGER,
                    metadata_scan_highestPPC INTEGER,
                    metadata_scan_averagePPC INTEGER,
                    metadata_scan_lowestPositionUncertainty REAL,
                    metadata_scan_highestPositionUncertainty REAL,
                    metadata_scan_averagePositionUncertainty REAL,
                    metadata_scan_pointcloudDensity INTEGER,
                    metadata_scan_polygonCount INTEGER,
                    collected_automatic_referencePhotoFilePath TEXT,
                    collected_automatic_tensorFlowTrainingFilePath TEXT,
                    collected_automatic_compressedImageSetFilePath TEXT,
                    collected_automatic_combinedPointcloudFilePath TEXT,
                    collected_automatic_finalMeshFilePath TEXT,
                    collected_automatic_slicedGcodeFilePath TEXT,
                    collected_manual_userComments TEXT
                )
            """)

# Collect Inputs
# TODO: Connect input methods
id = 0
grading_final_grade = ""
grading_metadata_flight_grade = 0.0
grading_metadata_process_grade = 0.0
grading_metadata_scan_grade = 0.0
grading_collected_automatic_grade = 0.0
metadata_flight_date = ""
metadata_flight_duration = ""
metadata_flight_parameters = ""
metadata_flight_rosLogs = ""
metadata_process_flight_success = 0
metadata_process_pointcloud_success = 0
metadata_process_mesh_success = 0
metadata_process_gcode_success = 0
metadata_process_imageRecognition_success = 0
metadata_process_processingTime_success = 0
metadata_scan_closestDistance = 0.0
metadata_scan_furthestDistance = 0.0
metadata_scan_averageDistance = 0.0
metadata_scan_lowestQuality = 0.0
metadata_scan_highestQuality = 0.0
metadata_scan_averageQuality = 0.0
metadata_scan_lowestPPC = 0  # PPC = Points per Capture
metadata_scan_highestPPC = 0
metadata_scan_averagePPC = 0
metadata_scan_lowestPositionUncertainty = 0.0
metadata_scan_highestPositionUncertainty = 0.0
metadata_scan_averagePositionUncertainty = 0.0
metadata_scan_pointcloudDensity = 0
metadata_scan_polygonCount = 0
collected_automatic_referencePhotoFilePath = ""
collected_automatic_tensorFlowTrainingFilePath = ""
collected_automatic_compressedImageSetFilePath = ""
collected_automatic_combinedPointcloudFilePath = ""
collected_automatic_finalMeshFilePath = ""
collected_automatic_slicedGcodeFilePath = ""
collected_manual_userComments = ""

# Convert data variables to a dictionary for convenience (map database column names to their corresponding values).
qdb_dict = {
    "id": id,
    "grading_final_grade": grading_final_grade,
    "grading_metadata_flight_grade": grading_metadata_flight_grade,
    "grading_metadata_process_grade": grading_metadata_process_grade,
    "grading_metadata_scan_grade": grading_metadata_scan_grade,
    "grading_collected_automatic_grade": grading_collected_automatic_grade,
    "metadata_flight_date": metadata_flight_date,
    "metadata_flight_duration": metadata_flight_duration,
    "metadata_flight_parameters": metadata_flight_parameters,
    "metadata_flight_rosLogs": metadata_flight_rosLogs,
    "metadata_process_flight_success": metadata_process_flight_success,
    "metadata_process_pointcloud_success": metadata_process_pointcloud_success,
    "metadata_process_mesh_success": metadata_process_mesh_success,
    "metadata_process_gcode_success": metadata_process_gcode_success,
    "metadata_process_imageRecognition_success": metadata_process_imageRecognition_success,
    "metadata_process_processingTime_success": metadata_process_processingTime_success,
    "metadata_scan_closestDistance": metadata_scan_closestDistance,
    "metadata_scan_furthestDistance": metadata_scan_furthestDistance,
    "metadata_scan_averageDistance": metadata_scan_averageDistance,
    "metadata_scan_lowestQuality": metadata_scan_lowestQuality,
    "metadata_scan_highestQuality": metadata_scan_highestQuality,
    "metadata_scan_averageQuality": metadata_scan_averageQuality,
    "metadata_scan_lowestPPC": metadata_scan_lowestPPC,
    "metadata_scan_highestPPC": metadata_scan_highestPPC,
    "metadata_scan_averagePPC": metadata_scan_averagePPC,
    "metadata_scan_lowestPositionUncertainty": metadata_scan_lowestPositionUncertainty,
    "metadata_scan_highestPositionUncertainty": metadata_scan_highestPositionUncertainty,
    "metadata_scan_averagePositionUncertainty": metadata_scan_averagePositionUncertainty,
    "metadata_scan_pointcloudDensity": metadata_scan_pointcloudDensity,
    "metadata_scan_polygonCount": metadata_scan_polygonCount,
    "collected_automatic_referencePhotoFilePath": collected_automatic_referencePhotoFilePath,
    "collected_automatic_tensorFlowTrainingFilePath": collected_automatic_tensorFlowTrainingFilePath,
    "collected_automatic_compressedImageSetFilePath": collected_automatic_compressedImageSetFilePath,
    "collected_automatic_combinedPointcloudFilePath": collected_automatic_combinedPointcloudFilePath,
    "collected_automatic_finalMeshFilePath": collected_automatic_finalMeshFilePath,
    "collected_automatic_slicedGcodeFilePath": collected_automatic_slicedGcodeFilePath,
    "collected_manual_userComments": collected_manual_userComments
}

# Add a new row of data into the Quality Database
sql_statement = f"INSERT INTO flights ({','.join(qdb_dict.keys())}) values({', '.join(['?']*len(qdb_dict))})"

try:
    db_connection.execute(sql_statement, tuple(qdb_dict.values()))
    db_connection.commit()
    print("Commit succes!!!! Wahoo!!!")
except:
    print("Commit failure :(")

view_statement = "SELECT * FROM flights"
view_qdb = pd.read_sql_query(view_statement, db_connection)
print(view_qdb)

db_connection.close()