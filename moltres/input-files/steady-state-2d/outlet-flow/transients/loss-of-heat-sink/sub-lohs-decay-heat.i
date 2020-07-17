pre_flow_velocity=112.75
pre_scale=1e-12    # precursor scaling factor

[GlobalParams]
  num_groups = 0
  num_precursor_groups = 8
  num_decay_heat_groups = 3
  temperature = temp
  use_exp_form = false
  group_fluxes = ''
  sss2_input = true
  pre_concs = 'pre1 pre2 pre3 pre4 pre5 pre6 pre7 pre8'
  account_delayed = true
  heat_concs = 'heat1 heat2 heat3'
  account_decay_heat = true
  decay_heat_fractions = '.0117 .0129 .0186'
  decay_heat_constants = '.1974 .0168 3.58e-4'
[]

[Mesh]
#  type = GeneratedMesh
#  dim = 1
#  nx = 2000
#  xmax = 225.5
#  elem_type = EDGE2
#  file = '../steady-state/msfr-923-K-temp-refine_out_loopApp0.e'
#  file = 'msfr-all-2d-core-decay-heat-923-K_out_loopApp0.e'
  file = '../../decay-heat/msfr-all-2d-core-decay-heat_out_loopApp0.e'
[../]

[Variables]
  [./temp]
    order = CONSTANT
    family = MONOMIAL
    scaling = 1e-3
#    initial_condition = 953
    initial_from_file_var = temp
    initial_from_file_timestep = LATEST
  [../]
  [./heat1]
    order = CONSTANT
    family = MONOMIAL
    scaling = 1
    initial_from_file_var = heat1
    initial_from_file_timestep = LATEST
  [../]
  [./heat2]
    order = CONSTANT
    family = MONOMIAL
    scaling = 1
    initial_from_file_var = heat2
    initial_from_file_timestep = LATEST
  [../]
  [./heat3]
    order = CONSTANT
    family = MONOMIAL
    scaling = 1
    initial_from_file_var = heat3
    initial_from_file_timestep = LATEST
  [../]
[]

[Precursors]
  [./core]
    var_name_base = pre
    outlet_boundaries = 'right'
    u_def = ${pre_flow_velocity}
    v_def = 0
    w_def = 0
    nt_exp_form = false
    family = MONOMIAL
    order = CONSTANT
    loop_precs = true
    multi_app = loopApp
    is_loopapp = true
    inlet_boundaries = 'left'
    scaling = ${pre_scale}
    init_from_file = true
  [../]
[]

[Kernels]
  [./temp_time_derivative]
    type = MatINSTemperatureTimeDerivative
    variable = temp
  [../]
  [./decay_heat1_time_deriv]
    type = ScalarTransportTimeDerivative
    variable = heat1
  [../]
  [./decay_heat1_decay]
    type = HeatPrecursorDecay
    variable = heat1
    decay_heat_group_number = 1
  [../]
  [./decay_heat2_time_deriv]
    type = ScalarTransportTimeDerivative
    variable = heat2
  [../]
  [./decay_heat2_decay]
    type = HeatPrecursorDecay
    variable = heat2
    decay_heat_group_number = 2
  [../]
  [./decay_heat3_time_deriv]
    type = ScalarTransportTimeDerivative
    variable = heat3
  [../]
  [./decay_heat3_decay]
    type = HeatPrecursorDecay
    variable = heat3
    decay_heat_group_number = 3
  [../]
  [./decay_heat_temp_source]
    type = DecayHeatSource
    variable = temp
  [../]
[]

[DGKernels]
  [./temp_adv]
    type = DGTemperatureAdvection
    variable = temp
    velocity = '${pre_flow_velocity} 0 0'
  [../]
  [./decay_heat1_convection]
    type = DGConvection
    velocity = '${pre_flow_velocity} 0 0'
    variable = heat1
  [../]
  [./decay_heat2_convection]
    type = DGConvection
    velocity = '${pre_flow_velocity} 0 0'
    variable = heat2
  [../]
  [./decay_heat3_convection]
    type = DGConvection
    velocity = '${pre_flow_velocity} 0 0'
    variable = heat3
  [../]
[]

[DiracKernels]
  [./heat_exchanger]
    type = DiracHX
    variable = temp
    power = 74133.6073445 
    point = '112.75 0 0'
  [../]
[]

[BCs]
  [./core_bottom]
    boundary = 'left'
    type = PostprocessorTemperatureInflowBC
    postprocessor = coreEndTemp
    variable = temp
    uu = ${pre_flow_velocity}
  [../]
  [./temp_advection_outflow]
    boundary = 'right'
    type = TemperatureOutflowBC
    variable = temp
    velocity = '${pre_flow_velocity} 0 0'
  [../]
  [./decay_heat1_inflow]
    boundary = 'left'
    type = PostprocessorInflowBC
    postprocessor = outlet_decay_heat1
    variable = heat1
    uu = ${pre_flow_velocity}
  [../]
  [./decay_heat1_outflow]
    type = OutflowBC
    velocity = '${pre_flow_velocity} 0 0'
    variable = heat1
    boundary = 'right'
  [../]
  [./decay_heat2_inflow]
    boundary = 'left'
    type = PostprocessorInflowBC
    postprocessor = outlet_decay_heat2
    variable = heat2
    uu = ${pre_flow_velocity}
  [../]
  [./decay_heat2_outflow]
    type = OutflowBC
    velocity = '${pre_flow_velocity} 0 0'
    variable = heat2
    boundary = 'right'
  [../]
  [./decay_heat3_inflow]
    boundary = 'left'
    type = PostprocessorInflowBC
    postprocessor = outlet_decay_heat3
    variable = heat3
    uu = ${pre_flow_velocity}
  [../]
  [./decay_heat3_outflow]
    type = OutflowBC
    velocity = '${pre_flow_velocity} 0 0'
    variable = heat3
    boundary = 'right'
  [../]
[]

[Functions]
  [./heatRemovalFcn]
    type = ParsedFunction
    value = '(tem-823)*370.6680367 * exp(-t)'
    vars = 'tem'
    vals = 'temp_ref'
  [../]
[]

[Controls]
  [./hxFuncCtrl]
    type = RealFunctionControl
    parameter = '*/*/power'
    function = 'heatRemovalFcn'
    execute_on = 'timestep_end'
  [../]
[]

[Materials]
  [./hx]
    type = GenericMoltresMaterial
    property_tables_root = '../../../../../data/xs-data-enrich/group/msfr_full_core_hx_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '1594 4.12487e-3 731.77'
  [../]
[]

[Executioner]
  type = Transient
  end_time = 20000

  nl_rel_tol = 1e-6
  nl_abs_tol = 1e-6

  solve_type = 'NEWTON'
  petsc_options = '-snes_converged_reason -ksp_converged_reason -snes_linesearch_monitor'
  petsc_options_iname = '-pc_type -pc_factor_shift_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu       NONZERO               superlu_dist'
  line_search = 'none'

  nl_max_its = 20
  l_max_its = 50

  dtmin = 1e-6
  [./TimeStepper]
    type = IterationAdaptiveDT
    dt = 1 # 1e-1
    cutback_factor = .5
    growth_factor = 10
    optimal_iterations = 25
  [../]
[]

[Preconditioning]
  [./SMP]
    type = SMP
    full = true
  [../]
[]

[Postprocessors]
  [./temp_loop]
    type = ElementAverageValue
    variable = temp
    outputs = 'console csv'
  [../]
  [./loopEndTemp]
    type = SideAverageValue
    variable = temp
    boundary = 'right'
  [../]
  [./coreEndTemp]
    type = Receiver
    default = 1023
  [../]
  [./loopMinTemp]
    type = ElementExtremeValue
    value_type = min
    variable = temp
    outputs = 'console'
  [../]
  [./temp_ref]
    type = PointValue
    variable = temp
    point = "112.6 0 0"
  [../]

  # Decay heat
  [./outlet_decay_heat1]
    type = Receiver
  [../]
  [./inlet_decay_heat1]
    type = SideAverageValue
    variable = heat1
    boundary = 'right'
  [../]
  [./outlet_decay_heat2]
    type = Receiver
  [../]
  [./inlet_decay_heat2]
    type = SideAverageValue
    variable = heat2
    boundary = 'right'
  [../]
  [./outlet_decay_heat3]
    type = Receiver
  [../]
  [./inlet_decay_heat3]
    type = SideAverageValue
    variable = heat3
    boundary = 'right'
  [../]
  [./average_decay_heat1]
    type = ElementAverageValue
    variable = heat1
    outputs = 'csv'
  [../]
  [./average_decay_heat2]
    type = ElementAverageValue
    variable = heat2
    outputs = 'csv'
  [../]
  [./average_decay_heat3]
    type = ElementAverageValue
    variable = heat3
    outputs = 'csv'
  [../]
[]

[Outputs]
  perf_graph = true
  csv = true
  print_linear_residuals = true
  [./exodus]
    type = Exodus
    execute_on = 'timestep_end'
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]
