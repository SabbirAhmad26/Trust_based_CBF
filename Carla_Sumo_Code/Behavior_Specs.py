from conflictCAVS import search_for_conflictCAVS_trustversion
from run_synchronization import dt

def Continuity(ego):
    v_max = 40
    u_max = 5.88
    maximum_displacement = 20

    if max(abs(ego['realpose'] - ego['prerealpose'])) >= maximum_displacement:
        score = 0
    else:
        score = 1

    return score


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
    return score

def constraint_test(simindex, que, table, order, k, ego, trust_threshold):
    # Constants
    threshold = 0
    multiple_constraints = 0
    stop = 0

    ip, index, position = search_for_conflictCAVS_trustversion(que, table, order, k, ego, multiple_constraints, trust_threshold)

    if simindex == 80 or ego['id'][1] == 9:
        stop = 1

    ultimate_score_rear = 0
    score_lateral = [0] * len(ego['lateralconstraint'])

    if ip and ego['rearendconstraint']:
        if min(ego['rearendconstraint']) >= threshold or ego['infeasiblity'] == 1 or que[ip]['scores'][1] == 0 or que[ip]['trust'][0] <= trust_threshold['low']:
            ultimate_score_rear = 1
    else:
        ultimate_score_rear = 1

    if ego['lateralconstraint'] and index:
        for i, lateral_constraint in enumerate(ego['lateralconstraint'], start=1):
            if lateral_constraint and index[i - 1][0] != -1:
                if lateral_constraint[-1] >= threshold or ego['infeasiblity'] == 1 or que[index[i - 1][0]]['scores'][1] == 0 or que[index[i - 1][0]]['trust'][0] <= trust_threshold['low']:
                    score_lateral[i - 1] = 1
            else:
                score_lateral[i - 1] = 1
    else:
        score_lateral = [1]

    ultimate_score_lateral = min(score_lateral)

    score = min(ultimate_score_lateral, ultimate_score_rear)

    return score


# Example usage
simindex_data = 80  # Replace with your actual data
que_data = [{}, {}, {}, {}]  # Replace with your actual data
table_data = None  # Replace with your actual data
order_data = None  # Replace with your actual data
k_data = None  # Replace with your actual data
ego_data = {
    'id': [0, 9],
    'rearendconstraint': [1, 2],  # Replace with your actual data
    'lateralconstraint': [[3, 4], [5, 6]],  # Replace with your actual data
    'infeasiblity': 0  # Replace with your actual data
}
trust_threshold_data = {'low': 0.1}  # Replace with your actual data

score_result, stop_result = constraint_test(simindex_data, que_data, table_data, order_data, k_data, ego_data, trust_threshold_data)
print("Score:", score_result)
print("Stop:", stop_result)



# Example usage
ego_data = {'state': [2.0, 1.0], 'prestate': [1.0, 0.0]}
dt = 0.1  # Set your global dt value here

score_result = dynamic_test(ego_data)
print("Score:", score_result)
