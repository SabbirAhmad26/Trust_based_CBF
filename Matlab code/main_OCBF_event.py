import time


def main_OCBF_event(i, car, mod, trust_threshold, MultipleConstraints, CAV_e):
    global dt, s3, u
    update_class_k_function = 0
    mitigation = 0
    trust = 1

    order = car['order']
    length_order = len(order)
    for k in range(length_order):
        car['que1'][order[k]]['txt']['String'] = str(k + 1)

        CAV_e['vel'][i][car['que1'][order[k]]['id'][1]] = car['que1'][order[k]]['state'][1]
        CAV_e['acc'][i][car['que1'][order[k]]['id'][1]] = car['que1'][order[k]]['state'][2]
        CAV_e['pos'][i][car['que1'][order[k]]['id'][1]] = car['que1'][order[k]]['state'][0]

        ip, index, position = search_for_conflictCAVS_trustversion(car['que1'], car['table'], car['order'], k,
                                                                   car['que1'][order[k]], MultipleConstraints,
                                                                   trust_threshold)

        car['que1'][order[k]]['see'] = Vision(car['que1'], car['que1'][order[k]], order[k])
        ip_seen = []

        for kkk in range(len(car['que1'][order[k]]['see'])):
            ind = car['que1'][order[k]]['see'][kkk]
            if car['que1'][ind]['id'][2] == car['que1'][order[k]]['id'][2]:
                if car['que1'][ind]['state'][0] >= car['que1'][order[k]]['state'][0]:
                    ip_seen = ind

        ilc = []

        flag_i, flag_i1, flag_ip, flag_ip_seen, flag_i_trust = Event_detector(car['que1'][order[k]], car['que1'], ip,
                                                                              ip_seen, index, CAV_e)
        if flag_i or flag_i1 or flag_ip or flag_ip_seen or flag_i_trust or (car['que1'][order[k]]['state'][0] == 0) or (
                car['que1'][order[k]]['state'][0] >= L):
            CAV_e['x_tk'][car['que1'][order[k]]['id'][1], 0][0] = car['que1'][order[k]]['state'][0]
            CAV_e['v_tk'][car['que1'][order[k]]['id'][1], 0][0] = car['que1'][order[k]]['state'][1]

            for kk in range(len(ip)):
                CAV_e['x_tk'][car['que1'][order[k]]['id'][1], 1][kk] = car['que1'][ip[kk]]['state'][0]
                CAV_e['v_tk'][car['que1'][order[k]]['id'][1], 1][kk] = car['que1'][ip[kk]]['state'][1]

            for kk in range(len(index)):
                for j in range(len(index[kk])):
                    if index[kk][j] == -1:
                        continue
                    else:
                        CAV_e['x_tk'][car['que1'][order[k]]['id'][1], 2 + kk - 1][j] = \
                            car['que1'][index[kk][j]]['state'][0]
                        CAV_e['v_tk'][car['que1'][order[k]]['id'][1], 2 + kk - 1][j] = \
                            car['que1'][index[kk][j]]['state'][1]

            if len(ip_seen):
                CAV_e['x_tk'][car['que1'][order[k]]['id'][1], 7][0] = car['que1'][ip_seen]['state'][0]
                CAV_e['v_tk'][car['que1'][order[k]]['id'][1], 7][0] = car['que1'][ip_seen]['state'][1]

            rt, CAV_e, infeasibility = OCBF_event(i, car['que1'][order[k]], car['que1'], ip, index, position, ilc,
                                                  CAV_e)
            CAV_e['counter'] += 1
            car['que1'][order[k]]['infeasibility'] = infeasibility
            CAV_e['buffer'][car['que1'][order[k]]['id'][1]] = rt

        else:
            u = car['que1'][order[k]]['state'][2]
            t = [0, dt]
            x0 = [
                car['que1'][order[k]]['state'][0],
                car['que1'][order[k]]['state'][1],
                car['que1'][order[k]]['state'][2]
            ]
            _, xx = odeint('second_order_model', x0[0:2], t, args=u)
            rt = [xx[-1, 0], xx[-1, 1], car['que1'][order[k]]['state'][2]]

            if rt[1] < 0:
                rt[1] = 0
                rt[0] = car['que1'][order[k]]['state'][0]

            if rt[0] - car['que1'][order[k]]['state'][0] < 0:
                time.sleep(1)

            CAV_e['buffer'][car['que1'][order[k]]['id'][1]] = rt

            if rt[0] <= car['que1'][order[k]]['metric'][3]:
                car['que1'][order[k]]['metric'][0] += dt
                if car['que1'][order[k]]['state'][2] >= 0:
                    car['que1'][order[k]]['metric'][1] = fuel_consumption(
                        car['que1'][order[k]]['metric'][1], rt[2], rt[1]
                    )
                else:
                    car['que1'][order[k]]['metric'][1] = car['que1'][order[k]]['metric'][1]

                car['que1'][order[k]]['metric'][2] += dt * 0.5 * rt[2] ** 2
            else:
                CAV_e['time'][car['que1'][order[k]]['id'][1], 0] = car['que1'][order[k]]['metric'][0]
                CAV_e['fuel'][car['que1'][order[k]]['id'][1], 0] = car['que1'][order[k]]['metric'][1]
                CAV_e['energy'][car['que1'][order[k]]['id'][1], 0] = car['que1'][order[k]]['metric'][2]
