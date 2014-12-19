#!/usr/bin/env python

import rospy
import smach
import smach_ros
import computer_sim
import random

#Average time it takes to perfectly navigate to the goal.
navigation_time = 60

#Average time time it takes to perfectly perform the task.
task_time = 60

# define state NavTele
class NavTele(smach.State):
    def __init__(self, c_sim):
        smach.State.__init__(self, outcomes=['COMPLETE'])
        self.c_sim = c_sim

    def execute(self, userdata):
        rospy.loginfo('Executing state NavTele')
        global navigation_time

        latency_cost = self.c_sim.latency * 5
        cpu_cost = self.c_sim.cpu * 1

        time_taken = random.gauss(navigation_time, 10)
        
        time_taken += latency_cost + cpu_cost

        return 'COMPLETE'


# define state NavShared
class NavShared(smach.State):
    def __init__(self, c_sim):
        smach.State.__init__(self, outcomes=['COMPLETE'])
        self.c_sim = c_sim

    def execute(self, userdata):
        rospy.loginfo('Executing state NavShared')
        global navigation_time

        latency_cost = self.c_sim.latency * 2
        cpu_cost = self.c_sim.cpu * 2

        time_taken = random.gauss(navigation_time, 5)
        
        time_taken += latency_cost + cpu_cost

        return 'COMPLETE'
        
# define state NavShared
class NavAuto(smach.State):
    def __init__(self, c_sim):
        smach.State.__init__(self, outcomes=['COMPLETE'])
        self.c_sim = c_sim

    def execute(self, userdata):
        rospy.loginfo('Executing state NavAuto')
        global navigation_time

        latency_cost = self.c_sim.latency * 1
        cpu_cost = self.c_sim.cpu * 2

        time_taken = random.gauss(navigation_time, 2)
        
        time_taken += latency_cost + cpu_cost

        failure_chance = random.random
        if failure_chance < 0.05:
            #If you failed, you have to teleop to the goal.
            latency_cost = self.c_sim.latency * 5
            cpu_cost = self.c_sim.cpu * 1

            time_taken += random.gauss(navigation_time, 2)
            
            time_taken += latency_cost + cpu_cost

            return 'COMPLETE'
        
        if failure_chance < 0.015:
            #If you 'kind-of' failed, you had to take longer to navigate.
            time_taken = time_taken * 1.5
             
            return 'COMPLETE'

# define state TaskOne
class TaskOne(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TaskOne')
        return 'COMPLETE'

# define state TaskTwo
class TaskTwo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TaskTwo')
        return 'COMPLETE'

# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['START'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        return 'START'




def main():
    c_sim = computer_sim.ComputerSim()
    c_sim.set_latency(0)
    c_sim.set_cpu(0)

    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'START':'START_NAV'})

        #Replace this sub SMACH state machine with an RL SMACH state machine.
        # Create the sub SMACH state machine        
        sm_nav = smach.StateMachineRL(outcomes=['NAV_COMPLETE', 'NAV_FAIL'], outcome_map={'NAV_COMPLETE':['COMPLETE'], 'NAV_FAIL':['ABORTED', 'FAILED']}, default_outcome='NAV_COMPLETE')

        with sm_nav:
            smach.StateMachineRL.add('NavTele', NavTele(c_sim))
            smach.StateMachineRL.add('NavShared', NavShared(c_sim))
            smach.StateMachineRL.add('NavAuto', NavAuto(c_sim))
        

        
        sm_task = smach.StateMachineRL(outcomes=['TASK_COMPLETE'], outcome_map={'TASK_COMPLETE':['COMPLETE']}, default_outcome='TASK_COMPLETE')
        # # Open the container
        with sm_task:
             # Add states to the container
            smach.StateMachineRL.add('TaskOne', TaskOne())
            smach.StateMachineRL.add('TaskTwo', TaskTwo())

        smach.StateMachine.add('START_NAV', sm_nav,
                                 transitions={'NAV_COMPLETE':'START_TASK', 'NAV_FAIL':'START_NAV'})

        smach.StateMachine.add('START_TASK', sm_task,
                                 transitions={'TASK_COMPLETE':'outcome5'})

    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()



if __name__ == '__main__':
    main()