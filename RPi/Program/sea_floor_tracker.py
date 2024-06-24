"""
Seafloor tracking class
block of x sea floor measurements are sent to class
measurements are sent to a cost function that finds a sp for a distance of 10 meters
when a new set point is created, the set_point is sent to the optimal path algorithm, this algorithm returns index(0) as the new set_point
"""
import queue

import numpy as np
from threading import Thread
from time import sleep


class SeafloorTracker(Thread):
    def __init__(self, length_rope, desired_distance, min_dist, dist_to_skip,
                 depth_of_rov, depth_beneath_boat, new_set_point_event, set_point_queue, alarm_list):
        Thread.__init__(self)
        self.length_rope = length_rope
        self.desired_distance = desired_distance
        self.min_dist = min_dist
        if desired_distance - min_dist >= dist_to_skip:
            self.dist_to_skip = dist_to_skip
        else:
            self.dist_to_skip = desired_distance - min_dist
        self.set_points = np.zeros([round(length_rope / 10)])
        self.depths_of_rov = depth_of_rov
        self.depths_beneath_boat = depth_beneath_boat
        self.new_set_point_event = new_set_point_event
        self.set_point_queue = set_point_queue
        self.array_count = 0
        self.alarm_list = alarm_list

    def run(self):
        while True:
            try:
                if self.new_set_point_event.is_set():
                    depths_beneath_rov = np.array(self.depths_beneath_boat.queue)
                    depths_of_rov = self.depths_of_rov
                    print("seafloor", self.set_points,self.i)
                    with self.depths_beneath_boat.mutex:
                        self.depths_beneath_boat.queue.clear()
                    setpoint = 'set_point_depth:{}'.format(self.get_set_point(depths_beneath_rov, depths_of_rov))
                    self.set_point_queue.put(setpoint)
                    self.new_set_point_event.clear()
                    self.i+=1

            except Exception as e:
                self.new_set_point_event.clear()
                print("erro: ", format(e))
                continue

    def get_set_point(self, sonar_values, depth_rov):
        """[Get a new set point based on the current set point and future depths]
        Args:
            sonar_values ([float]): [numpy 1D array with recorded sonar values]
            depth_rov ([float]): [The last measured ROV depth]
        Returns:
            [float]: [The new depth set point for the ROV]
        """
        #print("sonar_values:", sonar_values)
        current_set_point = self.set_points[0]

        #print("current_set_point:{}".format( current_set_point))
        new_set_point, alarm = self.__cost_function(sonar_values)
        #p#rint("new_set_point: ", new_set_point)
        #print("alarm: ", alarm)
        self.set_points = np.delete(self.set_points, 0)
        self.set_points = np.append(self.set_points, new_set_point)
        if self.set_points_full:
            self.set_points = self.__find_opt_sp(self.set_points, current_set_point, depth_rov, self.desired_distance,
                                                 self.min_dist, self.dist_to_skip)
        elif self.array_count >= len(self.set_points) - 1:
            self.set_points_full = True
        else:
            self.array_count = self.array_count + 1
        if alarm is True:
            self.set_points[0] = 0
        return self.set_points[0]

        return self.set_points[0]

    def set_paramter_values(self, length_rope=None, desired_distance=None, min_dist=None, dist_to_skip=None):
        """[summary]
        Args:
            length_rope ([int], optional): [The length of the towing cable]. Defaults to None.
            desired_distance ([int], optional): [The ROVs desired distance from the seafloor]. Defaults to None.
            min_dist ([int], optional): [The minimum distance the ROV can have to the seafloor]. Defaults to None.
            dist_to_skip ([int], optional): [The distance the ROV can have to the new set point before changing]. Defaults to None.
        """
        if length_rope is not None and length_rope != self.length_rope:
            new_size = round(length_rope / 10)
            self.set_points = self.__change_matrix_size(self.set_points, len(self.set_points), new_size)
        if desired_distance is not None:
            self.desired_distance = desired_distance
        if min_dist is not None and min_dist < self.desired_distance:
            self.min_dist = min_dist
        if dist_to_skip is not None:
            self.dist_to_skip = dist_to_skip
            if desired_distance - min_dist <= self.dist_to_skip:
                self.dist_to_skip = dist_to_skip
            else:
                self.dist_to_skip = desired_distance - min_dist

    def __cost_function(self, sonar_values):
        """[Run the echo sounder data through a cost function to find the optimal distance]
        Args:
            sonar_values ([float np array 1D]): [The recorded sonar values]
        Returns:
            [float]: [The optimal distance from the seafloor]
        """

        new_sp = 0
        alarm_flag = False
        max_set_point = round(min(sonar_values) - self.min_dist)
        print("max", max_set_point)
        if max_set_point >= 0:
            legal_set_points = np.arange(0, max_set_point, 0.5)
            cost = np.zeros([len(legal_set_points)])
            for index, set_point in enumerate(legal_set_points):
                for sonar_value in sonar_values:
                    cost[index] += abs(sonar_value - set_point - self.desired_distance)
            min_cost = np.amin(cost)
            print("min_cost",min_cost)
            min_cost_idx = np.array(np.argmax(cost == min_cost))
            new_sp = legal_set_points[min_cost_idx]
        else:
            alarm_flag = True
            for alarm in self.alarm_list:
                if alarm.get_sensor_name() == 'no_legal_sp':
                    alarm.set_alarm_value('True')
        return new_sp, alarm_flag

    def __find_opt_sp(self, set_points, current_sp, depth_rov, desired_distance, min_dist, dist_to_skip):
        """[Evaluetes the set points array to find an optimal path]
        Args:
            set_points ([type]): [description]
            current_sp ([type]): [description]
            depth_rov ([type]): [description]
        Returns:
            [type]: [description]
        """
        alarm = False
        set_points_mean = round(np.mean(set_points))
        #If the incline is too steep, set alarm = true and send message to GUI --> example
        if depth_rov-set_points[-1]>9999:
            alarm = True
            for alarm in self.alarm_list:
                if alarm.get_sensor_name() == 'incline_too_steep':
                    alarm.set_alarm_value('True')

        # If the ROV is on colision course every set point is set to a safe distance giving it time to go up
        if current_sp - set_points[-1] > desired_distance - min_dist or depth_rov - set_points[
            -1] > desired_distance - min_dist:
            set_points_min = min(set_points)
            set_points = np.empty(len(set_points))
            set_points.fill(set_points_min)
        elif current_sp - min(set_points) > desired_distance - min_dist or depth_rov - min(
                set_points) > desired_distance - min_dist:
            set_points[0] = min(set_points)
        # Rov closer to mean than current sp
        elif dist_to_skip > abs(depth_rov - set_points_mean) < abs(depth_rov - current_sp):
            set_points[0] = set_points_mean
        # if the distance between current sp and mean sp is greater than dist to ignore
        elif abs(current_sp - set_points_mean) > dist_to_skip:

            # if mean is less than current the ROV goes up
            if set_points_mean < current_sp:
                set_points[0] = set_points_mean
                # if the mean value is greater than current set point, and it wont reduce the depth if it have to go back up.
            elif set_points_mean - set_points[0] < desired_distance - min_dist and set_points_mean - min(
                    set_points) < desired_distance - min_dist:
                set_points[0] = set_points_mean
            else:
                set_points[0] = min(set_points)

        else:
            set_points[0] = current_sp
        return set_points
        
    def __change_matrix_size(self, matrix_to_change, size, new_size):
        difference = new_size - size
        if difference >= 1:
            idx = np.full((1, difference), matrix_to_change[-1])
            new_matrix = np.append(matrix_to_change, idx)
        elif difference <= -1:
            idx = np.arange(abs(difference))
            new_matrix = np.delete(matrix_to_change, idx)
        return new_matrix, alarm


if __name__ == "__main__":
    q1 = queue.Queue()
    q2 = queue.Queue()
    q3 = queue.Queue()
    q4 = queue.Queue()
    q1.put(15.01)
    q1.put(15.1)
    q1.put(15.2)

    q2.put(30.01)
    q2.put(30.9)
    q2.put(30.8)
    test = SeafloorTracker(300, 20, 9, 6, 10, q2, q3, q4)
    test.start()

    sleep(3)
    q3.put(True)
