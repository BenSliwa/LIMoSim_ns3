#include "scenarioregistry.h"

#include <iostream>

namespace LIMoSim {

ScenarioRegistry *ScenarioRegistry::getInstance()
{
    static ScenarioRegistry instance;
    return &instance;
}

void ScenarioRegistry::registerScenario(std::string _name, std::function<void(void)> _scenario)
{
    m_registry[_name] = _scenario;
}

Scenario ScenarioRegistry::getScenario(std::string _name)
{
    if (m_registry.count(_name)) {
        return m_registry.at(_name);
    } else {
        return nullptr;
    }
}

void ScenarioRegistry::printScenarios()
{
    for(auto entry : m_registry) {
        std::cout << "- " << entry.first << "\n";
    }
}

ScenarioRegistry::ScenarioRegistry()
{

}

} // namespace LIMoSim
