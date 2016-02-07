/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <string>

#include <boost/foreach.hpp>
#include <boost/python/stl_iterator.hpp>

namespace ecto_ros
{
  namespace bp = boost::python;

  using ecto::tendrils;
  struct Synchronizer
  {

    static void
    declare_params(tendrils& params)
    {
      params.declare<bp::object> ("subs", "A python dict ecto_Message_subscriber").required(true);
    }

    static void
    declare_io(const tendrils& p, tendrils& in, tendrils& out)
    {
      bp::object subs = p.get<bp::object> ("subs");
      if (!subs || subs == bp::object())
        return;
      bp::list l = bp::dict(subs).items();
      for (int j = 0; j < bp::len(l); ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        ecto::cell::ptr cell = bp::extract<ecto::cell::ptr>(bp::getattr(value,"__impl"));
        out.declare(keystring,cell->outputs["output"]);
      }
    }

    void
    configure(const tendrils& p, const tendrils& in, const tendrils& out)
    {
      ECTO_SCOPED_CALLPYTHON();
      bp::object subs = p.get<bp::object> ("subs");
      bp::list l = bp::dict(subs).items();
      for (int j = 0; j < bp::len(l); ++j)
      {
        bp::object key = l[j][0];
        bp::object value = l[j][1];
        std::string keystring = bp::extract<std::string>(key);
        ecto::cell::ptr cell = bp::extract<ecto::cell::ptr>(bp::getattr(value,"__impl"));
        cells_.push_back(cell);
        cell->configure();
      }
      // prep the helpers for processing
      unprocessed_cells.assign(cells_.begin(), cells_.end());
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      std::list<ecto::cell::ptr>::iterator iter;
      for(iter = unprocessed_cells.begin(); iter != unprocessed_cells.end();) {
        int value = (*iter)->process();
        if(value == ecto::QUIT) {
	    return ecto::QUIT;
        } else if (value == ecto::DO_OVER) {
            return ecto::DO_OVER;  // give the scheduler an opportunity to act (e.g. shutdown)
        } else if (value == ecto::OK) {
            iter = unprocessed_cells.erase(iter);
        }
      }
      // if we get here, all cells have been processed, clear them ready for the next run 
      unprocessed_cells.assign(cells_.begin(), cells_.end());
      return ecto::OK;
    }
    std::vector<ecto::cell::ptr> cells_;
  private:
    std::list<ecto::cell::ptr> unprocessed_cells;
  };
}

ECTO_CELL(ecto_ros_main, ecto_ros::Synchronizer, "Synchronizer",
    "Synchronizer synchronizes.")
