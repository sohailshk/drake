import unittest

import numpy as np

from pydrake.multibody.jupyter_widgets import (
    JointSliders,
    MakeJointSlidersThatPublishOnCallback,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import DiagramBuilder


class TestJupyterWidgets(unittest.TestCase):
    def _make_plant(self):
        plant = MultibodyPlant(time_step=0.0)
        Parser(plant).AddModelsFromString(
            """
<sdf version='1.9'>
  <model name='pendulum'>
    <link name='arm'/>
    <joint name='hinge' type='revolute'>
      <parent>world</parent>
      <child>arm</child>
      <axis><xyz>0 1 0</xyz></axis>
    </joint>
  </model>
</sdf>
""",
            "sdf",
        )
        plant.Finalize()
        return plant

    def test_joint_sliders_length(self):
        plant = self._make_plant()
        # A number is a number of pixels.
        dut = JointSliders(plant, length=300)
        self.assertEqual([x.layout.width for x in dut._slider], ["300px"])
        dut = JointSliders(plant, length=np.int64(300))
        self.assertEqual([x.layout.width for x in dut._slider], ["300px"])
        # A CSS string is used as-is.
        dut = JointSliders(plant, length="50%")
        self.assertEqual([x.layout.width for x in dut._slider], ["50%"])

    def test_publish_on_callback_length(self):
        plant = self._make_plant()
        builder = DiagramBuilder()
        builder.AddSystem(plant)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        for length, expected in [(123, "123px"), ("10em", "10em")]:
            sliders = MakeJointSlidersThatPublishOnCallback(
                plant, diagram, context, length=length
            )
            self.assertEqual([x.layout.width for x in sliders], [expected])
