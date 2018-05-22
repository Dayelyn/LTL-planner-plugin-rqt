from qt_gui.plugin import Plugin
from ltl_plannav_widget import LTLPlanNavWidget


class LTLPlugin(Plugin):

    def __init__(self, context):

        super(LTLPlugin, self).__init__(context)
        self._widget = LTLPlanNavWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.setObjectName('LTL Navigation Planner')

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
