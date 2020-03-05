#ifndef SCENARIOREGISTRY_H
#define SCENARIOREGISTRY_H

#include <string>
#include <map>
#include <functional>

namespace LIMoSim {

typedef std::function<void(void)> Scenario;

class ScenarioRegistry
{
public:
    static ScenarioRegistry* getInstance();

    void registerScenario(std::string _name, std::function<void(void)> _scenario);
    Scenario getScenario(std::string _name);
    void printScenarios();

private:
    ScenarioRegistry();

    std::map<std::string, std::function<void(void)>> m_registry;
};

} // namespace LIMoSim

#endif // SCENARIOREGISTRY_H
