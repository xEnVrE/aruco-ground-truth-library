/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PROBECONTAINER_H
#define PROBECONTAINER_H

#include <Probe.h>

#include <string>
#include <unordered_map>


class ProbeContainer
{
public:
    virtual ~ProbeContainer();

    bool is_probe(const std::string& name);

    Probe& get_probe(const std::string& name);

    void set_probe(const std::string& name, std::unique_ptr<Probe> probe);

protected:
    std::unordered_map<std::string, std::unique_ptr<Probe>> probes_;

    const std::string log_name_ = "ProbeContainer";
};

#endif /* PROBECONTAINER_H */
