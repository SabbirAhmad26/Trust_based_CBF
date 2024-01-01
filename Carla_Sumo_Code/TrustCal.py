import numpy as np
from vision import vision
from Behavior_Specs import dynamic_test, continuity, constraint_test


def calculate_trust(ego, simindex, que, table, trust_threshold):
    # Constants
    c = 5
    lambda_val = 0.95

    # Check conditions
    if sum(np.isnan(ego['scores'])) == 4 or ego['scores'][0] == 1:
        # Calculate scores
        score0 = continuity(ego)
        if score0 > 0:
            score1 = dynamic_test(ego)
            score2 = constraint_test(simindex, que, table, ego, trust_threshold)
            score3 = np.sign(len(vision(que, ego)))

            # Calculate trust
            scores = np.array([score0, score1, score2, score3])
            w = np.array([[0.6, 0.6, 0.6, 0.6], [1000, 10, 250, 0.25]])
            ego['reward'] = lambda_val * ego['reward'] + np.dot(w[0], scores)
            ego['regret'] = lambda_val * ego['regret'] + np.dot(w[1], 1 - scores)

            trust = ego['reward'] / (ego['reward'] + ego['regret'] + c)
        else:
            scores = np.array([0, 0, 0, 0])
            trust = 0
    else:
        scores = np.array([0, 0, 0, 0])
        trust = 0
        return scores, trust

    return scores, trust

