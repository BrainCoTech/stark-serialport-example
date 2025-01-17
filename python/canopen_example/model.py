class TouchStatus:
    def __init__(self, normal_forces, tangential_forces, tangential_directions, self_closes, mutual_close,):
        self.normal_force1 = normal_forces[0]
        self.normal_force2 = normal_forces[1]
        self.normal_force3 = normal_forces[2] if len(normal_forces) > 2 else 0
        self.tangential_force1 = tangential_forces[0]
        self.tangential_force2 = tangential_forces[1]
        self.tangential_force3 = tangential_forces[2] if len(tangential_forces) > 2 else 0
        self.tangential_direction1 = tangential_directions[0]
        self.tangential_direction2 = tangential_directions[1]
        self.tangential_direction3 = tangential_directions[2] if len(tangential_directions) > 2 else 0
        self.self_close1 = self_closes[0]
        self.self_close2 = self_closes[1] if len(self_closes) > 1 else 0
        self.mutual_close = mutual_close

    def __str__(self):
        return (
            f"\tnormal_force1: {self.normal_force1}, normal_force2: {self.normal_force2}, normal_force3: {self.normal_force3},\n"
            f"\ttangential_force1: {self.tangential_force1}, tangential_force2: {self.tangential_force2},\n"
            f"\ttangential_force3: {self.tangential_force3}, tangential_direction1: {self.tangential_direction1},\n"
            f"\ttangential_direction2: {self.tangential_direction2}, tangential_direction3: {self.tangential_direction3},\n"
            f"\tself_close1: {self.self_close1}, self_close2: {self.self_close2},\n"
            f"\tmutual_close: {self.mutual_close}"
        )


class TouchStatusData:
    def __init__(self, normal_forces, tangential_forces, tangential_directions):
        self.touch_status = []

        segments = [2, 3, 3, 3, 2]
        start = 0
        for segment in segments:
            self.touch_status.append(
                TouchStatus(
                    normal_forces[start:start+segment],
                    tangential_forces[start:start+segment],
                    tangential_directions[start:start+segment],
                    [0, 0],
                    0
                )
            )
            start += segment
        

    def __str__(self):
        return (
            f"touch_status[0]: \n{self.touch_status[0]},\n\n"
            f"touch_status[1]: \n{self.touch_status[1]},\n\n"
            f"touch_status[2]: \n{self.touch_status[2]},\n\n"
            f"touch_status[3]: \n{self.touch_status[3]},\n\n"
            f"touch_status[4]: \n{self.touch_status[4]}"
        )