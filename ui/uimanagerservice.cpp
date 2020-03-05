#include "uimanagerservice.h"

namespace LIMoSim {
namespace ui {

UiManagerService *UiManagerService::getInstance()
{
    static UiManagerService instance;
    return &instance;
}

UiManager *UiManagerService::getUiManager()
{
    return m_uiManager;
}

void UiManagerService::setUiManager(UiManager *_uiManager)
{
    m_uiManager = _uiManager;
    m_initialized = true;
}

bool UiManagerService::getUiDataInitialized()
{
    return m_uiDataInitialized;
}

void UiManagerService::setUiDataInitialized(bool _initialized)
{
    m_uiDataInitialized = _initialized;
}

UiManagerService::UiManagerService():
    m_initialized(false),
    m_uiDataInitialized(false)
{

}

} // namespace ui
} // namespace LIMoSim
