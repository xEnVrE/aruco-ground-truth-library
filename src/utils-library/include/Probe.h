/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PROBE_H
#define PROBE_H

#include <BayesFilters/Data.h>

#include <string>

class Probe
{
public:
    virtual ~Probe();

    void set_data(const bfl::Data&);

    bfl::Data& get_data();

protected:
    virtual void on_new_data() = 0;

private:
    bfl::Data data_;

    const std::string log_name_ = "Probe";
};

#endif /* PROBE_H */
