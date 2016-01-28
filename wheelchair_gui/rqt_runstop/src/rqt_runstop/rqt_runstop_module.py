import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from actionlib_msgs.msg import GoalID

class RunStopPlugin(Plugin):

    def __init__(self, context):
        super(RunStopPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RunStopPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_runstop'), 'src', 'rqt_runstop', 'runstop.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RunStopPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        # initialize publisher to publish cancel messages
        self._publisher = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)

        self._widget.runstopButton.clicked.connect(self.runstopButton_clicked)  # button handler


    # cancel all goals by sending empty goal to /move_base/cancel
    def runstopButton_clicked(self):
        emptyGoal = GoalID()
        self._publisher.publish(emptyGoal)
        return
