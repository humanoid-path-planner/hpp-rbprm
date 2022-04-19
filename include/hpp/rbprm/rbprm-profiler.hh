/*
Copyright (c) 2010-2013 Tommaso Urli

Tommaso Urli    tommaso.urli@uniud.it   University of Udine

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef RBPRM_PROFILER_H
#define RBPRM_PROFILER_H

#include "hpp/rbprm/utils/stop-watch.hh"

class RbPrmProfiler : public Stopwatch {
 public:
  /** Constructor */
  RbPrmProfiler(StopwatchMode _mode = NONE) : Stopwatch(_mode) {}

  /** Destructor */
  ~RbPrmProfiler() {}

  void add_to_count(const std::string& event, int nbOcc = 1) {
    std::map<std::string, int>::iterator it = count_.find(event);
    if (it == count_.end()) {
      count_.insert(std::make_pair(event, nbOcc));
    } else {
      it->second += nbOcc;
    }
  }

  void report_count(std::ostream& output = std::cout) {
    for (std::map<std::string, int>::iterator it = count_.begin();
         it != count_.end(); ++it) {
      output << it->first << ": " << it->second << std::endl;
    }
  }

  void report_all_and_count(int precision = 2,
                            std::ostream& output = std::cout) {
    report_all(precision, output);
    report_count(output);
  }

 private:
  std::map<std::string, int> count_;
};

RbPrmProfiler& getRbPrmProfiler();

#endif
