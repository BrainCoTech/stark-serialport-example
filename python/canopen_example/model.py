class TouchStatus:
    def __init__(self, normal_forces, tangential_forces, tangential_directions, self_proximities, mutual_proximity=None, is_edge: bool= True):
        self.normal_force1 = normal_forces[0]
        self.normal_force2 = normal_forces[1]
        self.normal_force3 = normal_forces[2] if not is_edge else None
        self.tangential_force1 = tangential_forces[0]
        self.tangential_force2 = tangential_forces[1]
        self.tangential_force3 = tangential_forces[2] if not is_edge else None
        self.tangential_direction1 = tangential_directions[0]
        self.tangential_direction2 = tangential_directions[1]
        self.tangential_direction3 = tangential_directions[2] if not is_edge else None
        self.self_proximity1 = self_proximities[0]
        self.self_proximity2 = self_proximities[1] if len(self_proximities) > 1 else None
        self.mutual_proximity = mutual_proximity
        self.is_edge = is_edge

    def __str__(self):
        result = (
            f"\tnormal_force1: {self.normal_force1}, normal_force2: {self.normal_force2}"
        )
        if not self.is_edge:
            result += f", normal_force3: {self.normal_force3}"

        result += (
            f"\n\ttangential_force1: {self.tangential_force1}, tangential_force2: {self.tangential_force2}"
        )
        if not self.is_edge:
            result += f", tangential_force3: {self.tangential_force3}"

        result += (
            f"\n\ttangential_direction1: {self.tangential_direction1}, tangential_direction2: {self.tangential_direction2}"
        )
        if not self.is_edge:
            result += f", tangential_direction3: {self.tangential_direction3}"

        if not self.is_edge:
            result += f"\n\tself_proximity1: {self.self_proximity1}, self_proximity2: {self.self_proximity2}"
        else:
            result += f"\n\tself_proximity1: {self.self_proximity1}" 
               
        if not self.is_edge:
            result += f"\n\tmutual_proximity: {self.mutual_proximity}"
        return result


class TouchStatusData:
    def __init__(self, normal_forces, tangential_forces, tangential_directions, self_proximities, mutual_proximities):
        self.touch_status = []

        segments = [2, 3, 3, 3, 2]
        start = 0
        start_1 = 0

        for i, segment in enumerate(segments):
            end = start + segment
            end_1 = start_1 + segment - 1

            mutual_proximity = None
            is_edge = True
            if i > 0 and i < 4:
                is_edge = False
                mutual_proximity = mutual_proximities[i-1]

            self.touch_status.append(
                TouchStatus(
                    normal_forces[start:end],
                    tangential_forces[start:end],
                    tangential_directions[start:end],
                    self_proximities[start_1:end_1],
                    mutual_proximity,
                    is_edge
                )
            )
            start = end
            start_1 = end_1
        

    def __str__(self):
        return (
            f"touch_status[Thumb]: \n{self.touch_status[0]}\n"
            f"touch_status[Index]: \n{self.touch_status[1]}\n"
            f"touch_status[Middle]: \n{self.touch_status[2]}\n"
            f"touch_status[Ring]: \n{self.touch_status[3]}\n"
            f"touch_status[Pinky]: \n{self.touch_status[4]}"
        )