pre_flow_velocity=112.75
pre_scale=1e-12    # precursor scaling factor

[GlobalParams]
  num_groups = 0
  num_precursor_groups = 8
  temperature = temp
  use_exp_form = false
  group_fluxes = ''
  sss2_input = true
  pre_concs = 'pre1 pre2 pre3 pre4 pre5 pre6 pre7 pre8'
  account_delayed = true
[]

[Mesh]
#  type = GeneratedMesh
#  dim = 1
#  nx = 2000
#  xmax = 225.5
#  elem_type = EDGE2
  file = '../../steady-state/msfr-all-2d_out_loopApp0.e'
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
[]

[DGKernels]
  [./temp_adv]
    type = DGTemperatureAdvection
    variable = temp
    velocity = '${pre_flow_velocity} 0 0'
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
[]

[Functions]
  [./heatRemovalFcn]
    type = ParsedFunction
    value = '(tem-823)*370.6680367'
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
    property_tables_root = '../../../../../data/xs-data-50pcm/group/msfr_full_core_hx_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '1594 4.12487e-3 731.77'
  [../]
[]

[Executioner]
  type = Transient
  end_time = 20

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
    outputs = 'exodus console'
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
    outputs = 'exodus console'
  [../]
  [./temp_ref]
    type = PointValue
    variable = temp
    point = "112.6 0 0"
  [../]
[]

[Outputs]
  perf_graph = true
  print_linear_residuals = true
  [./exodus]
    type = Exodus
    execute_on = 'timestep_end'
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]
