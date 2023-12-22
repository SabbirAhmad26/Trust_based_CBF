<<<<<<< HEAD
def Continuity(ego):
    v_max = 40
    u_max = 5.88
    maximum_displacement = 20

    if max(abs(ego['realpose'] - ego['prerealpose'])) >= maximum_displacement:
        score = 0
    else:
        score = 1

    return score

=======
>>>>>>> d15159632f821467f9214af0cd04d536ff96b332
global dt

def dynamic_test(ego):
    # Constants
    Umin = -6
    Umax = 3
    u_threshold = 0.5
    threshold = 0.3

    # Calculate MeasuredAcc and Measuredpos
    measured_acc = (ego['state'][1] - ego['prestate'][1]) / dt
    measured_pos = (ego['state'][0] - ego['prestate'][0])

    # Calculate Dynamicpos
    dynamic_pos = 0.5 * measured_acc * dt**2 + ego['prestate'][1] * dt

    # Check conditions
    if (measured_acc + u_threshold) >= Umin and (measured_acc - u_threshold) <= Umax:
        if measured_pos <= dynamic_pos + threshold and dynamic_pos - threshold <= measured_pos:
            score = 1
        else:
            score = 0
    else:
        score = 0

    # Check if score is 0 and set stop to 1
    if score == 0:
        stop = 1
    else:
        stop = 0

    return score


# Example usage
ego_data = {'state': [2.0, 1.0], 'prestate': [1.0, 0.0]}
dt = 0.1  # Set your global dt value here

score_result = dynamic_test(ego_data)
print("Score:", score_result)
<<<<<<< HEAD
>>>>>>> d15159632f821467f9214af0cd04d536ff96b332
=======
>>>>>>> d15159632f821467f9214af0cd04d536ff96b332
