// Copyright 2020 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>
//
/// Contains common ECC definitions and helper functions.

package ecc_pkg;

  // Calculate required ECC parity width:
  function get_parity_width;
    input integer data_width;
    integer cw_width;
    begin
      cw_width = 2;
      while ((1 << cw_width) < cw_width + data_width + 1)
        cw_width = cw_width + 1;
      get_parity_width = cw_width;
    end
  endfunction

  // Calculate required ECC codeword width:
  function get_cw_width;
    input integer data_width;
    begin
      get_cw_width = data_width + get_parity_width(data_width);
    end
  endfunction

endpackage
