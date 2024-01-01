import numpy as np

def vision(que, ego):
    # Constants
    precision_rate = 0.1  # Set your value for 'precision_rate' here
    R = 50

    indices = []
    ego_heading = np.arctan2(ego['realpose'][1] - ego['prerealpose'][1], ego['realpose'][0] - ego['prerealpose'][0])

    for i, other_car in enumerate(que):
        if que[i]['id'][1] == ego['id'][1] or que[i]['agent'] != 0:
            continue
        else:
            relative_heading = ego_heading - np.arctan2(-ego['prerealpose'][1] - 0.01 + other_car['realpose'][1],
                                                         -ego['prerealpose'][0] - 0.01 + other_car['realpose'][0])
            distance_squared = (ego['realpose'][0] - other_car['realpose'][0]) ** 2 + \
                               (ego['realpose'][1] - other_car['realpose'][1]) ** 2

            if distance_squared <= R ** 2 and -np.pi / 8 <= relative_heading <= np.pi / 8:
                #if perception_precision >= 1 - precision_rate:
                indices.append(i)

    return indices


# # Example usage
# que_data = [
#     {'realpose': [1.0, 2.0]},  # Replace with your actual data
#     {'realpose': [3.0, 4.0]},  # Replace with your actual data
#     # ... add more dictionaries as needed
# ]
#
# egocar_data = {'realpose': [5.0, 6.0], 'prerealpose': [3.0, 4.0]}  # Replace with your actual data
# k_data = 2  # Replace with your actual data
#
# indices_result = vision(que_data, egocar_data, k_data)
# print("Indices:", indices_result)
