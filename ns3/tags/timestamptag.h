#ifndef TIMESTAMPTAG_H
#define TIMESTAMPTAG_H


#include <ns3/core-module.h>
#include <ns3/tag.h>

namespace LIMoSim {
namespace NS3 {
namespace Tags {

using namespace ns3;

class TimestampTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  // these are our accessors to our tag structure
  void SetTimestamp (Time time);
  Time GetTimestamp (void) const;

  void Print (std::ostream &os) const;

private:
  Time m_timestamp;

  // end class TimestampTag
};

} // namespace Tags
} // namespace NS3
} // namespace LIMoSim

#endif // TIMESTAMPTAG_H
