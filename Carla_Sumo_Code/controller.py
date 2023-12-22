def  Event_detector(ego, que1, ip, ip_seen, index, CAV_e):

    # This function detects events for a CAV
    # Constants
    const1 = 0.1
    const2 = 0.2
    const3 = 0.3
    const4 = 0.4
    s1 = 0.5
    s2 = 0.6
    s3 = 0.7

    noise_term_position = max(abs(const1 * const2), abs(const1 * (1 - const2)))
    noise_term_speed = max(abs(const3 * const4), abs(const3 * (1 - const4)))

    flag_i = 0
    flag_i1 = 0
    flag_ip = 0
    flag_ip_seen = 0
    flag_i_trust = 0

    if ego["state"][1] >= CAV_e["v_tk"][ego["id"][0]][1][0] + s1 - noise_term_speed or \
            ego["state"][1] <= CAV_e["v_tk"][ego["id"][0]][1][0] - s1 + noise_term_speed or \
            ego["state"][0] >= CAV_e["x_tk"][ego["id"][0]][1][0] + s2 - noise_term_position or \
            ego["state"][0] <= CAV_e["x_tk"][ego["id"][0]][1][0] - s2 + noise_term_position:
        flag_i = 1

    for k in range(len(ip)):
        if que1[k]["state"][1] >= CAV_e["v_tk"][ego["id"][0]][2][k] + s1 - noise_term_speed or \
                que1[k]["state"][1] <= CAV_e["v_tk"][ego["id"][0]][2][k] - s1 + noise_term_speed or \
                que1[k]["state"][0] >= CAV_e["x_tk"][ego["id"][0]][2][k] + s2 - noise_term_position or \
                que1[k]["state"][0] <= CAV_e["x_tk"][ego["id"][0]][2][k] - s2 + noise_term_position:
            flag_ip = 1

        if que1[k]["trust"][0] >= que1[k]["trust"][1] + s3 or que1[k]["trust"][0] <= que1[k]["trust"][1] - s3:
            flag_i_trust = 1

    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                if que1[index[k][j]]["state"][1] >= CAV_e["v_tk"][ego["id"][0]][3 + k - 1][j] + s1 - noise_term_speed or \
                        que1[index[k][j]]["state"][1] <= CAV_e["v_tk"][ego["id"][0]][3 + k - 1][j] - s1 + noise_term_speed or \
                        que1[index[k][j]]["state"][0] >= CAV_e["x_tk"][ego["id"][0]][3 + k - 1][j] + s2 - noise_term_position or \
                        que1[index[k][j]]["state"][0] <= CAV_e["x_tk"][ego["id"][0]][3 + k - 1][j] - s2 + noise_term_position:
                    flag_i1 = 1

                if que1[index[k][j]]["trust"][0] >= que1[index[k][j]]["trust"][1] + s3 or \
                        que1[index[k][j]]["trust"][0] <= que1[index[k][j]]["trust"][1] - s3:
                    flag_i_trust = 1

    if ip_seen:
        if que1[ip_seen]["state"][1] >= CAV_e["v_tk"][ego["id"][0]][8][0] + s1 - noise_term_speed or \
                que1[ip_seen]["state"][1] <= CAV_e["v_tk"][ego["id"][0]][8][0] - s1 + noise_term_speed or \
                que1[ip_seen]["state"][0] >= CAV_e["x_tk"][ego["id"][0]][8][0] + s2 - noise_term_position or \
                que1[ip_seen]["state"][0] <= CAV_e["x_tk"][ego["id"][0]][8][0] - s2 + noise_term_position:
            flag_ip_seen = 1

    return flag_i,flag_i1,flag_ip,flag_ip_seen,flag_i_trust