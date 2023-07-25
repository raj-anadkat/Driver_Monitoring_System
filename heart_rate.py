import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import matplotlib.pyplot as plt

# Initialize Firebase Admin SDK
cred = credentials.Certificate("firebase/esp32demo-a6c5a-firebase-adminsdk-dni67-6ed9a3dbd0.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://esp32demo-a6c5a-default-rtdb.firebaseio.com'
})

# Reference to your Firebase Realtime Database
ref = db.reference('User1')

# Lists to store the data
timestamps = []
bpm_values = []
heart_rate_values = []

# Continuous data retrieval
while True:
    # Retrieve data
    data = ref.get()

    # Extract BPM and heart rate values
    if data:
        print(data)
        bpm = data.get('averageBPM')
        heart_rate = data.get('heartRate')
        timestamp = time.time()

        # Append values to the lists
        bpm_values.append(bpm)
        heart_rate_values.append(heart_rate)
        timestamps.append(timestamp)

        # Plotting
        plt.clf()  # Clear the previous plot

        # Plot average BPM vs time
        plt.subplot(2, 1, 1)
        plt.plot(timestamps, bpm_values, color='blue')
        plt.xlabel('Time')
        plt.ylabel('Average BPM')
        plt.title('Average BPM vs Time')

        # Plot heart rate vs time
        plt.subplot(2, 1, 2)
        plt.plot(timestamps, heart_rate_values, color='red')
        plt.xlabel('Time')
        plt.ylabel('Heart Rate')
        plt.title('Heart Rate vs Time')

        # Adjust the layout and display the plot
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)

    # Wait for some time before fetching data again (e.g., every 5 seconds)
    time.sleep(0.1)