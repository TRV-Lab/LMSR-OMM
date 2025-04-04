#ifndef _LOCALIZATION_DATA_H_
#define _LOCALIZATION_DATA_H_
#include "utils.h"

using namespace H5;
class Hypothesis
{
public:
    std::vector<double> lon;
    std::vector<double> lat;
    std::vector<double> heading;
    std::vector<float> probability;
    std::vector<uint8_t> valid;

};
class Cxx_localization
{
private:
    int length;
    std::vector<uint64_t> zeader_timestamp_ns;
    Hypothesis hypothesis[6];
    std::vector<uint8_t> map_sensor_mismatch_detected;
    std::vector<int32_t> state_machine_mode;
    Hypothesis best_hypothesis;
    std::vector<uint8_t> best_hypothesis_index;

public:
    void setLength(int l){length=l;}
    int getLength() const {return length;}
    std::vector<uint64_t>& getZeaderTimestampNs() {return zeader_timestamp_ns;}
    std::vector<uint8_t>& getMismatch() {return map_sensor_mismatch_detected;}
    std::vector<int32_t>& getStateMachineMode() {return state_machine_mode;}
    Hypothesis* const getHypothesis() {return hypothesis;}
    Hypothesis& getBestHypothesis() {return best_hypothesis;}
    std::vector<uint8_t>& getBestHypothesisIndex() {return best_hypothesis_index;}
    void bestHypothesis(bool allow_multi);
    void print()const;

};
class H5_localization
{
private:
    DataSet zeader_timestamp_ns;
    DataSet map_sensor_mismatch_detected;
    DataSet state_machine_mode;
    Group localization_data;
    void parseZeaderTimestampNsTo(Cxx_localization &) const;
    void parseMismatchTo(Cxx_localization &) const;
    void parseStateMachineModeTo(Cxx_localization &) const;
    void parseHypothesis(Cxx_localization &,std::string) const;
public:
    void getDataSet(H5File&);
    void parseTo(Cxx_localization &,bool allow_multi) const;
    DataSet getZeaderTimestampNs()const {return zeader_timestamp_ns;}
    DataSet getMismatch()const {return map_sensor_mismatch_detected;}    
    DataSet getStateMachineMode()const {return state_machine_mode;}
};
#endif