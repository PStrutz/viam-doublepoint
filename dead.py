        # # Calculate the mean of the initial measurements
        # mean_accel = np.mean(self.calibrate_acc, axis=0)

        # # Calculate the rotation angles required to align gravity with Z-axis
        # pitch = np.arctan2(-mean_accel[0], np.sqrt(mean_accel[1]**2 + mean_accel[2]**2))
        # roll = np.arctan2(mean_accel[1], mean_accel[2])

        # # Construct rotation matrices for pitch and roll
        # rotation_pitch = np.array([[1, 0, 0],
        #                         [0, np.cos(pitch), -np.sin(pitch)],
        #                         [0, np.sin(pitch), np.cos(pitch)]])

        # rotation_roll = np.array([[np.cos(roll), 0, np.sin(roll)],
        #                         [0, 1, 0],
        #                         [-np.sin(roll), 0, np.cos(roll)]])

        # # Combine the rotation matrices
        # self.rotation_matrix = np.dot(rotation_roll, rotation_pitch)

            # Normalize the vectors

        measured_vector = np.mean(self.calibrate_acc, axis=0)
        desired_vector = np.array([0, 0, -9.8])
        desired_vector /= np.linalg.norm(desired_vector)
        measured_vector /= np.linalg.norm(measured_vector)

        # Calculate the rotation axis and angle
        rotation_axis = np.cross(measured_vector, desired_vector)
        rotation_angle = np.arccos(np.dot(measured_vector, desired_vector))

        # Calculate the rotation matrix using the Rodrigues' rotation formula
        skew_matrix = np.array([
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0]
        ])
        self.rotation_matrix = np.identity(3) + np.sin(rotation_angle) * skew_matrix + (1 - np.cos(rotation_angle)) * np.dot(skew_matrix, skew_matrix)