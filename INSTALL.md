## UI Part

#### Qt 5.7
- Download and install Qt from https://www.qt.io/download/ (at least Qt 5.7 is required)
- Open the Qt-project file LIMoSim/ui/LIMoSim.pro
- Configure a suitable kit
- To enable ns3 coupling, the build switch `exts` must be set with the value `ns3`. This can be achieved as follows:
  - Open the `Projects` view on the right pane
  - Select `Build` under `Build & Run`
  - Add `exts=ns3` in the additional arguments input field located in the `Build Steps` section.
- Compile
- To start a standalone demo scenario:
  - Open the `Projects` view on the right pane
  - Select `Run` under `Build & Run`
  - Add `-n standalone -s $scenarioName` in the command line arguments input under `Run`
- To start a ns3 demo scenario:
  - Open the `Projects` view on the right pane
  - Select `Run` under `Build & Run`
  - Add `-n ns3 -s $scenarioName` in the command line arguments input under `Run`

  NB: Available Scenario names can be found here:
  - standalone: loadScenarioRegistry()@standalone/scenarios.css
  - ns3: loadScenarioRegistry()@ns3/examples/ns3examplescenarios.cc 