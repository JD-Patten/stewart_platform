#!/usr/bin/env python3
import math
import random
import numpy as np
import rclpy
import itertools
import copy
 
from robot.msg import PopulationStats, Organism, OrganismParameter

from rclpy.node import Node   
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from rosgraph_msgs.msg import Clock

# This is a "Simple Genetic Algorithm" 
# the Genes are paramaters of sin functions that corrospond to a degree of freedom 
# gene example:  amplitude of x, period of y, position offset of pitch...

use_planted_population = False

converted_parameters = []

# Iterate over each element in the planted_population list
for population in planted_population:
    # Extract the parameters for x, y, z, pitch, roll, and yaw
    x_params = [population['x'][0]['amplitude'], population['x'][0]['period'], population['x'][0]['position_offset'], population['x'][0]['time_offset'] / population['x'][0]['period']]
    y_params = [population['y'][0]['amplitude'], population['y'][0]['period'], population['y'][0]['position_offset'], population['y'][0]['time_offset'] / population['y'][0]['period']]
    z_params = [population['z'][0]['amplitude'], population['z'][0]['period'], population['z'][0]['position_offset'], population['z'][0]['time_offset'] / population['z'][0]['period']]
    pitch_params = [population['pitch'][0]['amplitude'], population['pitch'][0]['period'], population['pitch'][0]['position_offset'], population['pitch'][0]['time_offset']/ population['pitch'][0]['period']]
    roll_params =  [population['roll'][0]['amplitude'], population['roll'][0]['period'], population['roll'][0]['position_offset'], population['roll'][0]['time_offset']/ population['roll'][0]['period']]
    yaw_params =   [population['yaw'][0]['amplitude'], population['yaw'][0]['period'], population['yaw'][0]['position_offset'], population['yaw'][0]['time_offset']/ population['yaw'][0]['period']]

    # Append the parameters to the converted_parameters list
    converted_parameters.append([x_params, y_params, z_params, pitch_params, roll_params, yaw_params])
planted_population = converted_parameters


population_size = 100                        #number of organisms in each generation
if use_planted_population:
    population_size = len(converted_parameters)



    
servo_offset_1 = 95 * 0.001                 # mm to m Distance between servos 2-3, 4-5, 6-1
servo_offset_2 = 72 * 0.001                 # mm to m Distance between servos 1-2, 3-4, 5-6
servo_height = 43 * 0.001                   # mm to m 
    
end_connection_offset_1 = 50 * 0.001        # mm to m Distance between arm connection points 2-3, 4-5, 6-1
end_connection_offset_2 = 30 * 0.001        # mm to m Distance between arm connection points 1-2, 3-4, 5-6
end_connection_z_offset = -12.5 * 0.001     # mm to m 
    
draft_angle = 20                            #degrees
l1 = 65 * 0.001                             # mm to m 
    
    


                        #amplitude,   period,   position shift,   time shift
default_parameters =   [[0.0,         4.0,             0.0,              0.0 ],   #x
                        [0.0,         4.0,             0.0,              0.0 ],   #y
                        [0.0,         4.0,           200.0,              0.0 ],   #z
                        [0.0,         4.0,             0.0,              0.0 ],   #pitch
                        [0.0,         4.0,             0.0,              0.0 ],   #roll
                        [0.0,         4.0,             0.0,              0.0 ]]   #yaw

                        #amplitude,   period,   position shift,   time shift
limits =               [[[0,30],     [2.0,10.0],     [-25,25],       [0,1]],   #x
                        [[0,30],     [2.0,10.0],     [-25,25],       [0,1]],   #y
                        [[0,30],     [2.0,10.0],     [180,230],      [0,1]],   #z
                        [[0,20],     [2.0,10.0],     [-15,15],       [0,1]],   #pitch
                        [[0,20],     [2.0,10.0],     [-15,15],       [0,1]],   #roll
                        [[0,20],     [2.0,10.0],     [-15,15],       [0,1]]]   #yaw

ik_genes_limits = [[40,100], [20,50], [150,250]]



class SimulationManager(Node):
    def __init__(self):
        super().__init__("simulation_manager")

        self.use_ik_genes = False                         # this toggles using additional genes to control the dimensions of the end effector and long arms
        self.use_varied_speed = True
        self.use_constant_period = True
        self.publish_joint_goal_states = True

        if use_planted_population and len(planted_population[0]) > 6: 
            self.use_ik_genes = True


        self.mutation_frequency = 0.02                   #probability an organism has a random gene mutation
        self.crossover_frequency = 0.87                  #percent of next generation made by crossover 
        self.max_lifetime = 12                            #seconds
        self.ramp_up_time = 1                             #time for new organism to go from default parameters to it's own parameters
        self.influence_of_diversity_on_selection = 0.5    

        self.constant_period = 4.0

        self.l2 =  194 *0.001                             # arm length in mm

        self.running = False
        self.sub = self.create_subscription(Clock,"/clock", self.get_sim_time, 1) 
        self.sub = self.create_subscription(PoseStamped, "/model/skipper/pose", self.get_robot_pose_from_sim, 1)
        self.sub = self.create_subscription(JointState, "/world/bot_world/model/skipper/joint_state", self.get_joint_states,1)
        self.angles_from_sim = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.pub = self.create_publisher(Pose, "end_effector_pose", 1) 
        self.distance_traveled_pub = self.create_publisher(Float64, "/distance_traveled", 1)
        self.population_info_pub = self.create_publisher(PopulationStats, "/pop_info",1)
        self.lifetime_pub = self.create_publisher(Clock,"/lifetime",1)
        self.bot_pose_pub = self.create_publisher(String, "/bot_pose",1)

        self.pub1 = self.create_publisher(Float64, "/model/skipper/joint/arm1_joint/_0/cmd_pos", 1)  
        self.pub2 = self.create_publisher(Float64, "/model/skipper/joint/arm2_joint/_0/cmd_pos", 1)  
        self.pub3 = self.create_publisher(Float64, "/model/skipper/joint/arm3_joint/_0/cmd_pos", 1)  
        self.pub4 = self.create_publisher(Float64, "/model/skipper/joint/arm4_joint/_0/cmd_pos", 1)  
        self.pub5 = self.create_publisher(Float64, "/model/skipper/joint/arm5_joint/_0/cmd_pos", 1)  
        self.pub6 = self.create_publisher(Float64, "/model/skipper/joint/arm6_joint/_0/cmd_pos", 1) 

        self.joint_goal_pub = self.create_publisher(JointState,"/arm_control_input", 1)

        self.timer = self.create_timer(0.03, self.set_end_effector_pose)

        self.sim_time = 0.0
        self.sim_time_offset = 0.0

        self.initialize_transforms()
        self.organism_starting_position = [0.0, 0.0, 0.0]
        self.create_initial_population()

        self.running = True

    def get_joint_states(self, msg):
        self.angles_from_sim = msg.position

    def publish_pop_info(self):

        msg = PopulationStats()
        msg.current_organism = self.current_organism_number
        msg.generation_number = self.current_generation_number
        msg.population_size = population_size
        msg.population_fitness = float(sum(self.distances_traveled))

        msg.population_diversity = self.calculate_diversity(self.population)

        for i in range(len(self.population)):
            organism = Organism()  
            organism.max_distance = float(self.distances_traveled[i])

            parameter = OrganismParameter()
            parameter.amplitude =       self.population[i][0][0]
            parameter.position_offset = self.population[i][0][2]
            parameter.period =          self.population[i][0][1]
            parameter.time_offset =     self.population[i][0][3] * parameter.period
            organism.x.append(parameter)

            parameter = OrganismParameter()
            parameter.amplitude =       self.population[i][1][0]
            parameter.position_offset = self.population[i][1][2]
            parameter.period =          self.population[i][1][1]
            parameter.time_offset =     self.population[i][1][3] * parameter.period
            organism.y.append(parameter)

            parameter = OrganismParameter()
            parameter.amplitude =       self.population[i][2][0]
            parameter.position_offset = self.population[i][2][2]
            parameter.period =          self.population[i][2][1]
            parameter.time_offset =     self.population[i][2][3] * parameter.period
            organism.z.append(parameter)

            parameter = OrganismParameter()
            parameter.amplitude =       self.population[i][3][0]
            parameter.position_offset = self.population[i][3][2]
            parameter.period =          self.population[i][3][1]
            parameter.time_offset =     self.population[i][3][3] * parameter.period
            organism.pitch.append(parameter)

            parameter = OrganismParameter()
            parameter.amplitude =       self.population[i][4][0]
            parameter.position_offset = self.population[i][4][2]
            parameter.period =          self.population[i][4][1]
            parameter.time_offset =     self.population[i][4][3] * parameter.period
            organism.roll.append(parameter)

            parameter = OrganismParameter()
            parameter.amplitude =       self.population[i][5][0]
            parameter.position_offset = self.population[i][5][2]
            parameter.period =          self.population[i][5][1]
            parameter.time_offset =     self.population[i][5][3] * parameter.period
            organism.yaw.append(parameter)

            msg.organisms.append(organism)


        self.population_info_pub.publish(msg)
    
    def calculate_diversity(self, population):
        # finds the percent of unique genes in a population
        
        #population should be in the 'organism' format, see self.gene_sequence_to_organism()

        all_genes = []

        for organism in population:
            for dof in organism:
                all_genes.extend(dof)

        unique_genes = len(set(all_genes))

        diversity = unique_genes / len(all_genes) * 100

        return diversity

    def initialize_transforms(self):
    
        # set the translations for the servos and end effector connection points for inverse kinematics
        self.servo_translations = self.find_servo_or_connection_point_coords(servo_offset_1, servo_offset_2, servo_height)
        self.end_effector_arm_connections = self.find_servo_or_connection_point_coords(end_connection_offset_1, end_connection_offset_2, end_connection_z_offset)

        #the rotations are set up 'backwards' with yzx and negative signs because we are going from the servo to the base frame
        self.servo_rotations = [R.from_euler('zyx', [-120,  draft_angle,    0], degrees=True),
                                R.from_euler('zyx', [-120,  draft_angle, -180], degrees=True),
                                R.from_euler('zyx', [   0,  draft_angle,    0], degrees=True),
                                R.from_euler('zyx', [   0,  draft_angle, -180], degrees=True),
                                R.from_euler('zyx', [ 120,  draft_angle,    0], degrees=True),
                                R.from_euler('zyx', [ 120,  draft_angle, -180], degrees=True)]
    
    def create_initial_population(self):
        if use_planted_population:
            self.population = planted_population
            self.get_logger().info(f'Planted Initial Population: {self.population}') 
            
        else:
            self.population = []

            for i in range(population_size):
                validated = False
                while not validated:
                    potential_parameters = self.randomize_parameters()
                    speed_multiplier = self.check_parameters(potential_parameters)
                    if speed_multiplier != 0:
                        potential_parameters = self.vary_speed(potential_parameters, speed_multiplier) 
                        validated = True
                        

                self.population.append(potential_parameters)

                self.get_logger().info(f'Initial Population: {self.population}') 
        
        #initialize all the values to track this population and future populations
        self.current_generation_number = 0
        self.distances_traveled = [0] * population_size
        self.current_organism_number = 0
        self.at_starting_position = True
        self.organism_position = [0, 0, 0]
        self.past_generations = []
        self.past_distances_traveled = []

        self.publish_pop_info()

    def check_parameters(self, parameters):

        #returns 0 if the parametes are not solveable or go past the position limits 
        #if the parameters do not go past the angle limit and is solveable it returns a float
        # to scale the period and time offset parameters by to reach max velocity

        max_allowable_anglular_velocity = 1.0 #radians/second
        max_allowable_angle = 0.95  
        dt = .01
        max_angle = 0
        max_anglular_velocity = 0

        all_angles = []
        all_velocities = []


        if self.use_ik_genes:
            end_effector_connections = self.find_servo_or_connection_point_coords(parameters[6][0] * 0.001,
                                                                                  parameters[6][1] * 0.001,
                                                                                  end_connection_z_offset)
            l2 = parameters[6][2] * 0.001

        else:
            end_effector_connections = self.end_effector_arm_connections
            l2 = self.l2


        for i in range(int(4.1 / dt)):
            time = i * dt

            values = []
            for dof in range(6):
                amplitude =         parameters[dof][0]
                period =            parameters[dof][1]
                position_shift =    parameters[dof][2]
                time_shift =        parameters[dof][3] * period # multilying by period because this paramater is a percent of the period not a value in seconds

                value = amplitude * math.sin((math.pi * 2 / period) * (time + time_shift)) + position_shift
                values.append(value)
            
            x = values[0] * 0.001               #converts to meters
            y = values[1] * 0.001
            z = values[2] * 0.001

            q = quaternion_from_euler(math.radians(values[3]), math.radians(values[4]), math.radians(values[5]))
            try:
                angles = self.solve_inverse_kinematics(translation=[x,y,z],
                                                      quaternion=q,
                                                      l2=l2,
                                                      end_effector_connections=end_effector_connections)
            except:
                self.get_logger().info(f'Failed to solve IK: {parameters}')  
                return 0
            
            #see if there is a new max angle
            for angle in angles:
                if abs(angle) > max_angle:
                    max_angle = abs(angle)
            

            #set initial positions 
            if time == 0:
                last_angles = angles

            #calculate velocities and check for max velocity if time > 0
            else:
                velocities = [(angles[i] - last_angles[i]) / dt for i in range(len(angles))]

                for velocity in velocities:
                    if abs(velocity) > max_anglular_velocity:
                        max_anglular_velocity = abs(velocity)
                
                all_angles.append(angles)
                all_velocities.append(velocities)

                last_angles = angles
            
        
        if max_angle > max_allowable_angle:

            self.get_logger().info(f'Parameters not validated due too large angle: {parameters}')  

            return 0
        
        if max_anglular_velocity > max_allowable_anglular_velocity:

            if self.use_varied_speed:
                
                self.get_logger().info(f'Parameters validated but have too high angular velocity \n Parameters: {parameters} \n Angular velocity: {max_anglular_velocity}')  

                return max_anglular_velocity / max_allowable_anglular_velocity 
            
            else:
                self.get_logger().info(f'Parameters not validated due to too high angular velocity \n Parameters: {parameters} \n Angular velocity: {max_anglular_velocity}')  

                return 0

        
        self.get_logger().info(f'Validated parameters: {parameters}')  
        if self.use_varied_speed:
            return max_anglular_velocity / max_allowable_anglular_velocity
        else:
            return 1

    def vary_speed(self, parameters, speed_multiplier):
        #used to vary the periods and time offsets of an organisms parameters 

        # only vary first 6 sets of parameters because these are the sin waves for each dof (the rest are ik genes if being used)
        for dof in range(6):              
            parameters[dof][1] = parameters[dof][1] * speed_multiplier

        return parameters

    def get_sim_time(self, msg):
  
        if self.running == False:
            self.sim_time_offset = self.sim_time
        self.sim_time = msg.clock.sec + (msg.clock.nanosec * 0.000000001) 
        if self.sim_time - self.sim_time_offset > self.max_lifetime:
            self.sim_time_offset = self.sim_time
            self.end_of_life()

        lifetime = self.sim_time - self.sim_time_offset
        lifetime_msg = Clock() 
        lifetime_msg.clock.sec = int(lifetime)  
        lifetime_msg.clock.nanosec = int((lifetime - lifetime_msg.clock.sec) * 1e9)
        self.lifetime_pub.publish(lifetime_msg)
    
    def get_robot_pose_from_sim(self,msg):

        if msg.header.frame_id == 'bot_world':

            if self.at_starting_position:
                self.organism_starting_position = [msg.pose.position.x,
                                                    msg.pose.position.y,
                                                    msg.pose.position.z]
                self.at_starting_position = False

            self.organism_position = [msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z]
            
            x = msg.pose.position.x - self.organism_starting_position[0]
            y = msg.pose.position.y - self.organism_starting_position[1]

            distance_traveled_msg = Float64()
            distance_traveled_msg.data = math.sqrt(x**2 + y**2)
            self.distance_traveled_pub.publish(distance_traveled_msg)


            sim_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 0.000000001) 
            time_alive = sim_time - self.sim_time_offset

            bot_pose_msg = String()

            data = [
                sim_time,
                time_alive,
                self.current_generation_number, 
                self.current_organism_number, 
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
                self.angles_from_sim[0],
                self.angles_from_sim[1],
                self.angles_from_sim[2],
                self.angles_from_sim[3],
                self.angles_from_sim[4],
                self.angles_from_sim[5],
                self.end_effector_pose.position.x,
                self.end_effector_pose.position.y,
                self.end_effector_pose.position.z,
                self.end_effector_pose.orientation.x,
                self.end_effector_pose.orientation.y,
                self.end_effector_pose.orientation.z,
                self.end_effector_pose.orientation.w
            ]

            def format_float(value):
                if isinstance(value, float):
                    return f"{value:.5f}"
                return str(value)

            formatted_data = [format_float(x) for x in data]
            bot_pose_msg.data = ','.join(formatted_data)

            self.bot_pose_pub.publish(bot_pose_msg)

    def set_end_effector_pose(self):                     #finds the pose for the end effector given the sim time and the current organisms parameters
        if self.running:
            
            values = []

            for dof in range(6):
                amplitude =         self.population[self.current_organism_number][dof][0]
                period =            self.population[self.current_organism_number][dof][1]
                position_shift =    self.population[self.current_organism_number][dof][2]
                time_shift =        self.population[self.current_organism_number][dof][3] * period  # multilying by period because this paramater is a percent of the period not a value in seconds

                #instead of having the new organism jump straight into its position, it ramps up it's end effector position from it's default parameters
                time = self.sim_time - self.sim_time_offset
                if time < self.ramp_up_time:
                    amplitude *= (time) / self.ramp_up_time
                    position_shift = default_parameters[dof][2] + (position_shift - default_parameters[dof][2]) * ((time) / self.ramp_up_time)
                
                value = amplitude * math.sin((math.pi * 2 / period) * ((time) + time_shift)) + position_shift


                values.append(value)

            q = quaternion_from_euler(math.radians(values[3]),
                                      math.radians(values[4]),
                                      math.radians(values[5]))
            x = values[0] * 0.001               #converts to meters
            y = values[1] * 0.001
            z = values[2] * 0.001           


            #this is used for the bot_pose_pub that saves the info for blender animations
            # this doesn't work probably because theres a delay between the goal pose and results from the sim
            self.end_effector_pose = Pose()
            self.end_effector_pose.position.x = x
            self.end_effector_pose.position.y = y
            self.end_effector_pose.position.z = z
            self.end_effector_pose.orientation.x = q[0]
            self.end_effector_pose.orientation.y = q[1]
            self.end_effector_pose.orientation.z = q[2]
            self.end_effector_pose.orientation.w = q[3]


            try:
                angles = self.solve_inverse_kinematics(translation=[x,y,z], quaternion= q)
            except:
                self.get_logger().info(f'Failed to solve IK on:  {self.population[self.current_organism_number]} \n for sim time:  {time}')  
            
            try:
                msg1 = Float64()
                msg1.data = angles[0]
                
                msg2 = Float64()
                msg2.data = angles[1]

                msg3 = Float64()
                msg3.data = angles[2]

                msg4 = Float64()
                msg4.data = angles[3]

                msg5 = Float64()
                msg5.data = angles[4]

                msg6 = Float64()
                msg6.data = angles[5]

                self.pub1.publish(msg1)
                self.pub2.publish(msg2)
                self.pub3.publish(msg3)
                self.pub4.publish(msg4)
                self.pub5.publish(msg5)
                self.pub6.publish(msg6)

            except:
                self.get_logger().info(f'Failed to publish angle messages')  

            if self.publish_joint_goal_states:
              try:
                  msg = JointState()
                  msg.position = angles
                  self.joint_goal_pub.publish(msg)
              except: 
                  self.get_logger().info(f'Failed to publish angle goals JS messages')

    def end_of_life(self): 
        self.running = False
        distance_traveled = math.sqrt((self.organism_position[0] - self.organism_starting_position[0])**2 +
                                      (self.organism_position[1] - self.organism_starting_position[1])**2 +
                                      (self.organism_position[2] - self.organism_starting_position[2])**2)
        
        self.distances_traveled[self.current_organism_number] = distance_traveled

        self.at_starting_position = True

        self.get_logger().info(f'Distances traveled: {self.distances_traveled}') 


        if self.current_organism_number == (population_size - 1):       #check if last organism in population
            self.new_generation()
            self.current_organism_number = 0
            self.current_generation_number += 1
            self.distances_traveled = [0] * population_size

            if self.use_ik_genes:
                self.end_effector_arm_connections = self.find_servo_or_connection_point_coords(self.population[self.current_organism_number][6][0] * 0.001,
                                                                                               self.population[self.current_organism_number][6][1] * 0.001,
                                                                                               end_connection_z_offset)
                self.l2 = self.population[self.current_organism_number][6][2] * 0.001

            self.publish_pop_info()
            self.running = True
            

        else:
            self.current_organism_number += 1

            if self.use_ik_genes:
                self.end_effector_arm_connections = self.find_servo_or_connection_point_coords(self.population[self.current_organism_number][6][0] * 0.001,
                                                                                               self.population[self.current_organism_number][6][1] * 0.001,
                                                                                               end_connection_z_offset)
                self.l2 = self.population[self.current_organism_number][6][2] * 0.001

            self.publish_pop_info()
            self.running = True
                  
    def new_generation(self):

        self.past_generations.append(self.population)
        self.past_distances_traveled.append(self.distances_traveled)

        #select parents to crossover
        number_of_parents = int(self.crossover_frequency * population_size)
        if number_of_parents % 2 != 0: number_of_parents -= 1

        parents = self.select_organisms(self.population, self.distances_traveled, number_of_parents, self.influence_of_diversity_on_selection)

        self.get_logger().info(f'selected parents: {parents}')

        
        #create a random set of pairings for the parents
        numbers = list(range(number_of_parents))
        random.shuffle(numbers)
        pairings = [[numbers[i], numbers[i+1]] for i in range(0, number_of_parents, 2)]
        self.get_logger().info(f'parent pairings: {pairings}')

        #create offspring 
        offspring = []
        for pair in pairings:
            child1, child2 = self.crossover(parents[pair[0]], parents[pair[1]])
            offspring.append(child1)
            offspring.append(child2)

        # add mutations to a small percentage of the offspring
        mutated_offspring = []
        for organism in offspring:
            if random.random() < self.mutation_frequency:
                mutated_organism = self.mutate(organism)
                mutated_offspring.append(mutated_organism)
                self.get_logger().info(f'mutated organism from {organism} to: {mutated_organism}')  
            else:
                mutated_offspring.append(organism)

        self.get_logger().info(f'mutated_offspring {mutated_offspring}')

        # select organisms to continue to the next generation
        number_to_survive = population_size - number_of_parents
        self.get_logger().info(f'number of surviving organisms: {number_to_survive}')


        self.get_logger().info(f'self.population before selecting the surviving orgs: {self.population}')

        surviving_organisms = self.select_organisms(self.population, self.distances_traveled, number_to_survive, diversity_influence= 0.0)
        self.get_logger().info(f'surviving organisms: {surviving_organisms}')

        #create new population
        self.population = mutated_offspring + surviving_organisms

        self.get_logger().info(f'New Population: {self.population}')
        return
    
    def select_organisms(self,population,fitness_scores, number_to_select, diversity_influence):
        # This function selects a group of organisms from a population
        # Each organisms' probability of selection is based on its fitness, and how it will contribute to
        # making the current selection more diverse. 
        # In the metaphor of a Roulette wheel, the roulette wheel is broken up into two portions,
        # the diversity portion and fitness portion, the size of these portions can be varied
        # In the fitness portion, each organism is given a slice of the roulette wheel porportional in size to its fitness
        # In the diversity portion, each organism is given a slice of the roulette wheel proportional to how much it would increase 
        # the total diversity of the current selection. If no organism will increase the diversity of the
        # population, only the fitness portion is considered
        # On the first selection, only the fitness of the organisms are considered

        # make copies of these so they are not affected outside of this function
        population_copy = copy.deepcopy(population)
        fitness_scores_copy = copy.deepcopy(fitness_scores)

        diversity_portion = diversity_influence
        fitness_portion = 1 - diversity_portion

        #select the first parent from fitness alone
        total_fitness = sum(fitness_scores_copy)
        probabilities = [fitness / total_fitness for fitness in fitness_scores_copy]

        #'spin the roulette wheel' and select the first parent
        selected_organism = random.choices(population_copy, weights= probabilities, k=1)[0]
        selected_parents = [selected_organism]
        self.get_logger().info(f'first selection made: {selected_parents}')  

        # remove the selection from the population_copy so it is not selected again
        index = population_copy.index(selected_organism)
        population_copy.remove(selected_organism)
        fitness_scores_copy.pop(index)
        self.get_logger().info(f'updated selection pool: {population_copy} \n updated fitnesses: {fitness_scores_copy}')  

        # find the rest of the parentss

        for _ in range(number_to_select - 1):
            
            #calculate the probabilities for the diversity portion of the roulette wheel
            
            self.get_logger().info(f'calculating diversity of : {selected_parents + [selected_parents[0]]}') 

            current_selection_diversity = self.calculate_diversity(selected_parents + [selected_parents[0]])
            diversity_increases = []

            for i, organism in enumerate(population_copy):

                #find the increase in diversity by adding this organism
                self.get_logger().info(f'calculating diversity of : {selected_parents + [organism]}')  

                selection_with_organism = selected_parents

                diversity_with_current_organism = self.calculate_diversity(selected_parents + [organism])
                diversity_increase = diversity_with_current_organism - current_selection_diversity
                diversity_increases.append(diversity_increase)
            
            # normalize the diversity increases to a list of probabilities
            total_diversity_increases = sum(diversity_increases)
            if total_diversity_increases == 0:
                fitness_probabilities_only = True
            else:
                fitness_probabilities_only = False
                probabilities_from_diversity = [diversity_score / total_diversity_increases for diversity_score in diversity_increases]

            # calculate the probabilities for the fitness portion of the roulette wheel
            total_fitness = sum(fitness_scores_copy)                
            probabilities_from_fitness = [fitness / total_fitness for fitness in fitness_scores_copy]

            #combine diversity and fiteness probabilities 
            if fitness_probabilities_only:
                probabilities = probabilities_from_fitness
            else:
                probabilities = [(p_d * diversity_portion) + (p_f * fitness_portion) for p_d, p_f in zip(probabilities_from_diversity, probabilities_from_fitness)]

            #'spin the roulette wheel' and select the first parent
            selected_organism = random.choices(population_copy, weights= probabilities, k=1)[0]
            selected_parents.append(selected_organism)

            # remove the selection from the population_copy so it is not selected again
            index = population_copy.index(selected_organism)
            population_copy.pop(index)
            fitness_scores_copy.pop(index)
            self.get_logger().info(f'updated selection pool: {population_copy} \n updated fitnesses: {fitness_scores_copy}')  

        return selected_parents
    
    def crossover(self, parent1, parent2):

        #if a constant period is being used reset the constant period
        #That way if the parents both had their periods changed by vary_speed() their different periods  
        # will not combine

        if self.use_constant_period:
            for dof in range(6):
                parent1[dof][1] = self.constant_period
                parent2[dof][1] = self.constant_period
        
        #flatten parents paramaters to be a single list of genes
        parent1_genes = self.organism_to_gene_sequence(parent1)
        parent2_genes = self.organism_to_gene_sequence(parent2)

        crossover_points = list(range(1,len(parent1_genes)))   # 6 dofs * 4 parameters = 24 genes and 23 cut points
        random.shuffle(crossover_points)


        # apply one-point crossover to create 2 offspring from 2 parents and verify validity of offspring
        for cut in crossover_points:
            offspring1_genes = parent1_genes[:cut] + parent2_genes[cut:]
            offspring2_genes = parent2_genes[:cut] + parent1_genes[cut:]

            self.get_logger().info(f'trying one-point crossover at cut point on:\n {parent1}\n and:\n {parent2}\n at crossover point: {cut}')

            # switch back to 2d matrix of sin wave function paramaters
            offspring1 = self.gene_sequence_to_organism(offspring1_genes)
            offspring2 = self.gene_sequence_to_organism(offspring2_genes)

            #If the first offspring isn't valid don't bother checking the second, try a different crossover point
            speed_multiplier = self.check_parameters(offspring1)

            if speed_multiplier == 0: 
                continue  
            else: 
                offspring1 = self.vary_speed(offspring1, speed_multiplier) 


            #check if the cut point works for 
            speed_multiplier = self.check_parameters(offspring2)
            if speed_multiplier != 0:
                offspring2 = self.vary_speed(offspring2, speed_multiplier) 
                return offspring1, offspring2


        #try applying two-point crossover if none of the one-point crossover solutions worked
        crossover_point_combinations = [(min(combo), max(combo)) for combo in itertools.combinations(crossover_points, 2)]
        for cut_pairs in crossover_point_combinations:
            cut1 = cut_pairs[0]
            cut2 = cut_pairs[1]

            offspring1_genes = parent1_genes[:cut1] + parent2_genes[cut1:cut2] + parent1_genes[cut2:]
            offspring2_genes = parent2_genes[:cut1] + parent1_genes[cut1:cut2] + parent2_genes[cut2:]

            self.get_logger().info(f'trying two-point crossover at cut point on:\n {parent1}\n and:\n {parent2}\n at crossover points: {cut1}, {cut2}')

            # switch back to 2d matrix of sin wave function paramaters
            offspring1 = self.gene_sequence_to_organism(offspring1_genes)
            offspring2 = self.gene_sequence_to_organism(offspring2_genes)

            #If the first offspring isn't valid don't bother checking the second, try different crossover points
            speed_multiplier = self.check_parameters(offspring1)
            if speed_multiplier == 0: 
                continue  
            else: 
                offspring1 = self.vary_speed(offspring1, speed_multiplier) 

            #check if the cut point works for 
            speed_multiplier = self.check_parameters(offspring2)
            if speed_multiplier != 0:
                offspring2 = self.vary_speed(offspring2, speed_multiplier) 
                return offspring1, offspring2
            

            
        
        # if somehow two point crossover didn't work either just return the parents as the offspring
        self.get_logger().info(f'could not find a viable solution with one or two-point crossover, returning the parents as offspring' )
        return parent1, parent2
   
    def mutate(self, organism):
        #select random parameter in random dof to mutate
        found_valid_mutation = False
        while not found_valid_mutation:

            dof_index = random.randint(0,len(organism)-1)
            parameter_index = random.randint(0,3)

            if self.use_constant_period and parameter_index == 1 and dof_index < 6 : continue          #make sure not to mutate the period if were using a constant period

            if parameter_index >= len(organism[dof_index]) : continue              #Make sure the set of genes is long enough for the index

            if dof_index < 6:
                organism[dof_index][parameter_index] = random.uniform(limits[dof_index][parameter_index][0],limits[dof_index][parameter_index][1])
            else:
                organism[dof_index][parameter_index] = random.uniform(ik_genes_limits[parameter_index][0],ik_genes_limits[parameter_index][1])

            speed_multiplier = self.check_parameters(organism)

            if speed_multiplier != 0:
                organism = self.vary_speed(organism, speed_multiplier)
                found_valid_mutation = True

        return organism
    
    def organism_to_gene_sequence(self, organism):

        # I have two ways to represent an individual

        # The first is the "organism" or "parameters" that is a list of dofs [x,y,z,pitch,roll,yaw], each
        # dof being a list of parameters that control a sin_wave [amplitude, period, 
        # position shift, time shift]
        # The second is a "gene_sequence" which is the 2d matrix of an "organism" flattend down
        # into a single list of genes

        gene_sequence = []

        for dof in organism:
            gene_sequence.extend(dof)

        return gene_sequence
    
    def gene_sequence_to_organism(self, gene_sequence):

        # I have two ways to represent an individual

        # The first is the "organism" or "parameters" that is a list of dofs [x,y,z,pitch,roll,yaw], each
        # dof being a list of parameters that control a sin_wave [amplitude, period, 
        # position shift, time shift]
        # The second is a "gene_sequence" which is the 2d matrix of an "organism" flattend down
        # into a single list of genes

        number_of_dof = 6
        genes_per_dof = 4
        number_of_ik_genes = 3
        organism = []

        for i in range(number_of_dof):
            dof = []
            for j in range(genes_per_dof):
                gene_index = i * genes_per_dof + j
                dof.append(gene_sequence[gene_index])
            organism.append(dof)

        if self.use_ik_genes:
            ik_genes = []
            for i in range(number_of_ik_genes):
                ik_genes.append(gene_sequence[number_of_dof * genes_per_dof + i])
            organism.append(ik_genes)

        return organism
        
    def solve_inverse_kinematics(self, translation, quaternion, l2 = None, end_effector_connections = None):   

        if l2 == None:
            l2 = self.l2

        if end_effector_connections == None:
            end_effector_connections = self.end_effector_arm_connections

        angles = []

        for arm_number in range(6):

            #rotate the end connection point

            rotation_matrix = R.from_quat(quaternion)
            end_connection_point = rotation_matrix.apply(end_effector_connections[arm_number])

            #translate the end connection point so it's at the correct position in the base frame

            end_connection_point += translation

            #rotate and translate the point so it is in the servo's frame

            end_connection_point -= self.servo_translations[arm_number]
            end_connection_point = self.servo_rotations[arm_number].apply(end_connection_point)

            x = end_connection_point[0]
            y = end_connection_point[1]
            z = end_connection_point[2]


            #adding and subtracting the acos portion decides which of the two possible solutions to choose
            #using if z < 0 uses different solutions for the upside down servos. this keeps the arms pointing outwards
            
            if z < 0:
                angle = math.atan2(z,y) + math.acos((l2**2 - y**2 - z**2 - x**2 - l1**2) / (-2 * l1 * math.sqrt(y**2 + z**2)))
            else:
                angle = math.atan2(z,y) - math.acos((l2**2 - y**2 - z**2 - x**2 - l1**2) / (-2 * l1 * math.sqrt(y**2 + z**2)))
        
            angles.append(angle)

        return angles
            
    def find_servo_or_connection_point_coords(self, offset_1, offset_2, z_height):

        d1 = 1/6 * (offset_1 - offset_2)
        d2 = 1/3 * (offset_1 - offset_2)
        c = offset_1 - (2*d1)
        radius = math.sqrt(c**2 + (d2)**2 - 2*c*d2 * math.cos(math.radians(60)))

        rotate_120 = R.from_euler('z', 120, degrees=True)
        rotate_240 = R.from_euler('z', 240, degrees=True)

        number1_translations = [-1 * math.sqrt((radius ** 2) - ((offset_1/2) **2)), offset_1/2, z_height]
        number6_translations = [number1_translations[0], -1 * number1_translations[1], z_height]
        number2_translations = rotate_240.apply(number6_translations)
        number3_translations = rotate_240.apply(number1_translations)
        number4_translations = rotate_120.apply(number6_translations)
        number5_translations = rotate_120.apply(number1_translations)


        translations = [number1_translations, number2_translations, number3_translations,
                        number4_translations, number5_translations, number6_translations]
        
        return translations
    
    def randomize_parameters(self):                 #get random values for amplitude, period, ... for each dof within set limits

        random_parameters = []

        for dof in range(6):
            values = []
            for value in range(4):
                values.append(random.uniform(limits[dof][value][0],limits[dof][value][1]))
            random_parameters.append(values)

        if self.use_constant_period:
            for dof in random_parameters:
                dof[1] = self.constant_period

        if self.use_ik_genes:
            ik_genes = []
            for gene in range(3):
                ik_genes.append(random.uniform(ik_genes_limits[gene][0],ik_genes_limits[gene][1]))
            random_parameters.append(ik_genes)
              

        return random_parameters

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
            
def main(args=None):
    rclpy.init()                        
    node = SimulationManager()      

    try:
        rclpy.spin(node)              #Starts the node and keeps it running untill it's interrupted by the user
    except KeyboardInterrupt:
        print("Terminating Node... ")
        node.destroy_node()


if __name__ == '__main__':
    main()