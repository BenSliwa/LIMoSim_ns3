#include "deliverysettingsservice.h"

#include <iostream>

#include "LIMoSim/mobility/car/strategic/truckdelivery.h"

namespace LIMoSim {
namespace delivery {

DeliverySettingsService *DeliverySettingsService::getInstance()
{
    static DeliverySettingsService instance;
    return &instance;
}

const DeliverySettingVariant *DeliverySettingsService::getDeliverySetting(DeliverySettingKey _settingKey)
{
    if(m_settings.count(_settingKey)) {
        return &m_settings.at(_settingKey);
    } else {
        return nullptr;
    }
}

void DeliverySettingsService::setDeliverySetting(DeliverySettingKey _settingKey, DeliverySettingVariant _setting)
{
    if (m_settings.count(_settingKey)) {
        std::cout << "DeliverySettingsService:setDeliverySetting overriding setting " << _settingKey << std::endl;
    }
    m_settings[_settingKey] = _setting;
}

DeliverySettingsService::DeliverySettingsService()
{
    // Setting defaults
    m_settings[DRONE_INTERACTION_MODE] = LIMoSim::DroneInteractionMode::ONSITE;
    m_settings[TRUCK_UNLOADING_DELAY] = 120;
    m_settings[UAV_DOCKING_DELAY] = 10;
    m_settings[UAV_LAUNCH_DELAY] = 10;
    m_settings[UAV_UNLOADING_TIME] = 25;
}

} // namespace delivery
} // namespace LIMoSim
