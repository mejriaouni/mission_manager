#ifndef DONE_MISSIONCONTAINER_H
#define DONE_MISSIONCONTAINER_H

#include <any>
#include <map>
#include <string>

namespace done
{

/**
 * Class used to store the data for the missions
 */
class MissionContainer
{
public:
  /**
   * A constructor.
   */
  MissionContainer();

  /**
   * A function to add data to component container
   *
   * @param name of the data element that we add
   * @param data element that will be added to storage
   */
  template <typename ValueT>
  void addData(const std::string name, const ValueT& data);

  /**
   * A function that returns stored data
   *
   * @param name of the stored data element
   * @return stored data element
   */
  template <typename ValueT>
  ValueT getData(const std::string name) const;

private:
  // map that stores missions
  std::map<std::string, std::any> mMissionContainer;

};

template <typename ValueT>
void MissionContainer::addData(const std::string name, const ValueT& data)
{
  mMissionContainer[name] = data;
}

template <typename ValueT>
ValueT MissionContainer::getData(const std::string name) const
{
  return std::any_cast<ValueT>(mMissionContainer.at(name));
}

}
#endif // DONE_MISSIONCONTAINER_H
