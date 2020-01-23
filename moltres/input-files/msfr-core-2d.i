flow_velocity=112.75 # cm/s
pre_flow_velocity=112.75
nt_scale=1e-15     # neutron flux scaling factor
pre_scale=1e-9    # precursor scaling factor
ini_temp=923     # initial temp
diri_temp=923    # dirichlet BC temp
ini_neut=1e14

[GlobalParams]
  num_groups = 6
  num_precursor_groups = 8
  num_decay_heat_groups = 3
  temperature = temp
  group_fluxes = 'group1 group2 group3 group4 group5 group6'
  pre_concs = 'pre1 pre2 pre3 pre4 pre5 pre6 pre7 pre8'
  heat_concs = 'heat1 heat2 heat3'
  use_exp_form = false
  sss2_input = true
  account_delayed = true
  account_decay_heat = true
  decay_heat_fractions = '.0117 .0129 .0186'
  decay_heat_constants = '.1974 .0168 3.58e-4'
  integrate_p_by_parts = true
  gravity = '0 0 0'
  pspg = true
  supg = true
  alpha = 1
[]

[Mesh]
  file = 'msfr-flow-only_exodus.e'
[../]

[Problem]
  type = FEProblem
  coord_type = RZ
  rz_coord_axis = Y
[]

[Variables]
  [./ux]
    family = LAGRANGE
    order = FIRST
    initial_from_file_var = ux
    initial_from_file_timestep = LATEST
  [../]
  [./uy]
    family = LAGRANGE
    order = FIRST
    initial_from_file_var = uy
    initial_from_file_timestep = LATEST
  [../]
  [./p]
    family = LAGRANGE
    order = FIRST
    initial_from_file_var = p
    initial_from_file_timestep = LATEST
  [../]
  [./group1]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
  [../]
  [./group2]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
  [../]
  [./group3]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
  [../]
  [./group4]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
  [../]
  [./group5]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
  [../]
  [./group6]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
  [../]
  [./temp]
    order = FIRST
    family = LAGRANGE
    scaling = 1e-3
  [../]

  # Decay heat
  [./heat1]
    order = CONSTANT
    family = MONOMIAL
    block = 'fuel'
    scaling = 1
  [../]
  [./heat2]
    order = CONSTANT
    family = MONOMIAL
    block = 'fuel'
    scaling = 1
  [../]
  [./heat3]
    order = CONSTANT
    family = MONOMIAL
    block = 'fuel'
    scaling = 1
  [../]
[]

[Precursors]
  [./pres]
    var_name_base = pre
    block = 'fuel'
    outlet_boundaries = 'outlet'
    # prec_scale = 1
    constant_velocity_values = false
    uvel = ux
    vvel = uy
    nt_exp_form = false
    family = MONOMIAL
    order = CONSTANT
    transient = true
    loop_precs = true
    multi_app = loopApp
    is_loopapp = false
    inlet_boundaries = 'inlet'
    scaling = ${pre_scale}
    # jac_test = true
  [../]
[]

[Kernels]
  [./mass]
    type = INSMassRZ
    variable = p
    u = ux
    v = uy
    p = p
    # pspg = true
    # alpha = '.333'
  [../]
  [./x_time_deriv]
    type = INSMomentumTimeDerivative
    variable = ux
  [../]
  [./y_time_deriv]
    type = INSMomentumTimeDerivative
    variable = uy
  [../]
  [./x_momentum_space]
    type = INSMomentumTractionFormRZ
    variable = ux
    u = ux
    v = uy
    p = p
    component = 0
    # supg = true
    # alpha = 1
  [../]
  [./y_momentum_space]
    type = INSMomentumTractionFormRZ
    variable = uy
    u = ux
    v = uy
    p = p
    component = 1
    # supg = true
    # alpha = 1
  [../]
  # Neutronics
  [./time_group1]
    type = NtTimeDerivative
    variable = group1
    group_number = 1
  [../]
  [./time_group2]
    type = NtTimeDerivative
    variable = group2
    group_number = 2
  [../]
  [./time_group3]
    type = NtTimeDerivative
    variable = group3
    group_number = 3
  [../]
  [./time_group4]
    type = NtTimeDerivative
    variable = group4
    group_number = 4
  [../]
  [./time_group5]
    type = NtTimeDerivative
    variable = group5
    group_number = 5
  [../]
  [./time_group6]
    type = NtTimeDerivative
    variable = group6
    group_number = 6
  [../]

  [./diff_group1]
    type = GroupDiffusion
    variable = group1
    group_number = 1
  [../]
  [./diff_group2]
    type = GroupDiffusion
    variable = group2
    group_number = 2
  [../]
  [./diff_group3]
    type = GroupDiffusion
    variable = group3
    group_number = 3
  [../]
  [./diff_group4]
    type = GroupDiffusion
    variable = group4
    group_number = 4
  [../]
  [./diff_group5]
    type = GroupDiffusion
    variable = group5
    group_number = 5
  [../]
  [./diff_group6]
    type = GroupDiffusion
    variable = group6
    group_number = 6
  [../]

  [./sigma_r_group1]
    type = SigmaR
    variable = group1
    group_number = 1
  [../]
  [./sigma_r_group2]
    type = SigmaR
    variable = group2
    group_number = 2
  [../]
  [./sigma_r_group3]
    type = SigmaR
    variable = group3
    group_number = 3
  [../]
  [./sigma_r_group4]
    type = SigmaR
    variable = group4
    group_number = 4
  [../]
  [./sigma_r_group5]
    type = SigmaR
    variable = group5
    group_number = 5
  [../]
  [./sigma_r_group6]
    type = SigmaR
    variable = group6
    group_number = 6
  [../]

  [./inscatter_group1]
    type = InScatter
    variable = group1
    group_number = 1
  [../]
  [./inscatter_group2]
    type = InScatter
    variable = group2
    group_number = 2
  [../]
  [./inscatter_group3]
    type = InScatter
    variable = group3
    group_number = 3
  [../]
  [./inscatter_group4]
    type = InScatter
    variable = group4
    group_number = 4
  [../]
  [./inscatter_group5]
    type = InScatter
    variable = group5
    group_number = 5
  [../]
  [./inscatter_group6]
    type = InScatter
    variable = group6
    group_number = 6
  [../]

  [./fission_source_group1]
    type = CoupledFissionKernel
    variable = group1
    group_number = 1
  [../]
  [./fission_source_group2]
    type = CoupledFissionKernel
    variable = group2
    group_number = 2
  [../]
  [./fission_source_group3]
    type = CoupledFissionKernel
    variable = group3
    group_number = 3
  [../]
  [./fission_source_group4]
    type = CoupledFissionKernel
    variable = group4
    group_number = 4
  [../]
  [./fission_source_group5]
    type = CoupledFissionKernel
    variable = group5
    group_number = 5
  [../]
  [./fission_source_group6]
    type = CoupledFissionKernel
    variable = group6
    group_number = 6
  [../]

  [./delayed_group1]
    type = DelayedNeutronSource
    variable = group1
    group_number = 1
    block = 'fuel'
  [../]
  [./delayed_group2]
    type = DelayedNeutronSource
    variable = group2
    group_number = 2
    block = 'fuel'
  [../]
  [./delayed_group3]
    type = DelayedNeutronSource
    variable = group3
    group_number = 3
    block = 'fuel'
  [../]
  [./delayed_group4]
    type = DelayedNeutronSource
    variable = group4
    group_number = 4
    block = 'fuel'
  [../]
  [./delayed_group5]
    type = DelayedNeutronSource
    variable = group5
    group_number = 5
    block = 'fuel'
  [../]
  [./delayed_group6]
    type = DelayedNeutronSource
    variable = group6
    group_number = 6
    block = 'fuel'
  [../]

  # Temperature
  [./temp_all]
    type = INSTemperature
    variable = temp
    u = ux
    v = uy
  [../]
  [./temp_time]
    type = INSTemperatureTimeDerivative
    variable = temp
  [../]
  [./temp_source]
    type = TransientFissionHeatSource
    nt_scale=1
    variable = temp
  [../]

  # Decay heat
  [./decay_heat1_time_deriv]
    type = ScalarTransportTimeDerivative
    variable = heat1
    block = 'fuel'
  [../]
  [./decay_heat1_source]
    type = HeatPrecursorSource
    variable = heat1
    decay_heat_group_number = 1
    block = 'fuel'
  [../]
  [./decay_heat1_decay]
    type = HeatPrecursorDecay
    variable = heat1
    decay_heat_group_number = 1
    block = 'fuel'
  [../]
  [./decay_heat2_time_deriv]
    type = ScalarTransportTimeDerivative
    variable = heat2
    block = 'fuel'
  [../]
  [./decay_heat2_source]
    type = HeatPrecursorSource
    variable = heat2
    decay_heat_group_number = 2
    block = 'fuel'
  [../]
  [./decay_heat2_decay]
    type = HeatPrecursorDecay
    variable = heat2
    decay_heat_group_number = 2
    block = 'fuel'
  [../]
  [./decay_heat3_time_deriv]
    type = ScalarTransportTimeDerivative
    variable = heat3
    block = 'fuel'
  [../]
  [./decay_heat3_source]
    type = HeatPrecursorSource
    variable = heat3
    decay_heat_group_number = 3
    block = 'fuel'
  [../]
  [./decay_heat3_decay]
    type = HeatPrecursorDecay
    variable = heat3
    decay_heat_group_number = 3
    block = 'fuel'
  [../]
  [./decay_heat_temp_source]
    type = DecayHeatSource
    variable = temp
    block = 'fuel'
  [../]
[]

[DGKernels]
  [./diff_pre1]
    type = DGDiffusion
    variable = pre1
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre2]
    type = DGDiffusion
    variable = pre2
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre3]
    type = DGDiffusion
    variable = pre3
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre4]
    type = DGDiffusion
    variable = pre4
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre5]
    type = DGDiffusion
    variable = pre5
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre6]
    type = DGDiffusion
    variable = pre6
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre7]
    type = DGDiffusion
    variable = pre7
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]
  [./diff_pre8]
    type = DGDiffusion
    variable = pre8
    diff = 0.47 # 4e-1 / .85
    epsilon = -1
    sigma = 6
  [../]

  # Decay heat
  [./decay_heat1_convection]
    type = DGCoupledAdvection
    variable = heat1
    uvel = ux
    vvel = uy
    block = 'fuel'
  [../]
  [./decay_heat2_convection]
    type = DGCoupledAdvection
    variable = heat2
    uvel = ux
    vvel = uy
    block = 'fuel'
  [../]
  [./decay_heat3_convection]
    type = DGCoupledAdvection
    variable = heat3
    uvel = ux
    vvel = uy
    block = 'fuel'
  [../]
[]

[Materials]
  [./fuel]
    type = GenericMoltresMaterial
    property_tables_root = '../mc-paper-moltres/data/mc_paper_fuel_'
    interp_type = 'spline'
    prop_names = 'cp rho k mu'
    prop_values = '1555 4.125e-3 732 4e-1'
    # prop_values = '1355 4.125e-3 .010097 1.0146e-4'
    block = 'fuel'
  [../]
  # [./rho_fuel]
  #   type = DerivativeParsedMaterial
  #   f_name = rho
  #   function = '(4983.56 - .882 * temp) * .000001'    # kg cm-3
  #   args = 'temp'
  #   derivative_order = 1
  #   block = 'fuel'
  # [../]
  # [./k_fuel]
  #   type = ParsedMaterial
  #   f_name = k
  #   function = '(0.928 + 8.397e-5 * temp) * .01'    # J cm-1 K-1
  #   args = 'temp'
  #   block = 'fuel'
  # [../]
  # [./mu_fuel]
  #   type = ParsedMaterial
  #   f_name = mu
  #   function = 'rho * exp(3689 / temp) * 5.55e-8 * 10000'
  #   material_property_names = 'rho'
  #   args = 'temp'
  #   block = 'fuel'
  # [../]
[]

[Functions]
  [./velFunc]
    type = ParsedFunction
    # value = '-4 * (y / 18.75 - (y / 18.75)^2) * (1 - exp(-t/20))'
    # value = '-508.5025 * 4 * (y / 18.75 - (y / 18.75)^2) * (1 - exp(-t/5))'
    value = '-508.5025 * 4 * (y / 18.75 - (y / 18.75)^2) * (1)'
  [../]
[]

[BCs]
  [./ux_dirichlet]
    type = DirichletBC
    boundary = 'outer bottom top axial'
    variable = ux
    value = 0
  [../]
  [./uy_dirichlet]
    type = DirichletBC
    boundary = 'outer inlet top bottom'
    variable = uy
    value = 0
  [../]
  [./ux_inlet]
    type = FunctionDirichletBC
    boundary = 'inlet'
    variable = ux
    function = 'velFunc'
  [../]
  [./temp_core_inlet]
    boundary = 'inlet'
    type = PostprocessorDirichletBC
    postprocessor = inlet_mean_temp
    variable = temp
  [../]
  [./vacuum_group1]
    type = VacuumConcBC
    boundary = 'outer bottom top axial inlet outlet'
    variable = group1
  [../]
  [./vacuum_group2]
    type = VacuumConcBC
    boundary = 'outer bottom top axial inlet outlet'
    variable = group2
  [../]
  [./vacuum_group3]
    type = VacuumConcBC
    boundary = 'outer bottom top axial inlet outlet'
    variable = group3
  [../]
  [./vacuum_group4]
    type = VacuumConcBC
    boundary = 'outer bottom top axial inlet outlet'
    variable = group4
  [../]
  [./vacuum_group5]
    type = VacuumConcBC
    boundary = 'outer bottom top axial inlet outlet'
    variable = group5
  [../]
  [./vacuum_group6]
    type = VacuumConcBC
    boundary = 'outer bottom top axial inlet outlet'
    variable = group6
  [../]
  [./pre1_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre1
    uvel = ux
    vvel = uy
  [../]
  [./pre2_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre2
    uvel = ux
    vvel = uy
  [../]
  [./pre3_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre3
    uvel = ux
    vvel = uy
  [../]
  [./pre4_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre4
    uvel = ux
    vvel = uy
  [../]
  [./pre5_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre5
    uvel = ux
    vvel = uy
  [../]
  [./pre6_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre6
    uvel = ux
    vvel = uy
  [../]
  [./pre7_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre7
    uvel = ux
    vvel = uy
  [../]
  [./pre8_outlet]
    type = CoupledOutflowBC
    boundary = 'outlet'
    variable = pre8
    uvel = ux
    vvel = uy
  [../]

  # Decay heat
  [./decay_heat1_inflow]
    type = PostprocessorCoupledInflowBC
    postprocessor = inlet_decay_heat1
    variable = heat1
    uvel = ux
    vvel = uy
    boundary = 'inlet'
  [../]
  [./decay_heat1_outflow]
    type = CoupledOutflowBC
    variable = heat1
    uvel = ux
    vvel = uy
    boundary = 'outlet'
  [../]
  [./decay_heat2_inflow]
    type = PostprocessorCoupledInflowBC
    postprocessor = inlet_decay_heat2
    variable = heat2
    uvel = ux
    vvel = uy
    boundary = 'inlet'
  [../]
  [./decay_heat2_outflow]
    type = CoupledOutflowBC
    variable = heat2
    uvel = ux
    vvel = uy
    boundary = 'outlet'
  [../]
  [./decay_heat3_inflow]
    type = PostprocessorCoupledInflowBC
    postprocessor = inlet_decay_heat3
    variable = heat3
    uvel = ux
    vvel = uy
    boundary = 'inlet'
  [../]
  [./decay_heat3_outflow]
    type = CoupledOutflowBC
    variable = heat3
    uvel = ux
    vvel = uy
    boundary = 'outlet'
  [../]
[]

[Executioner]
  type = Transient
  end_time = 100

 nl_rel_tol = 1e-9
 nl_abs_tol = 1e-6

  solve_type = 'NEWTON'
  petsc_options = '-snes_converged_reason -ksp_converged_reason -snes_linesearch_monitor'
  petsc_options_iname = '-pc_type -pc_factor_shift_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu       NONZERO       superlu_dist'
  line_search = 'none'
  # petsc_options_iname = '-pc_type -pc_asm_overlap -sub_pc_type -sub_pc_factor_levels'
  # petsc_options_value = 'asm      2               ilu          2'

  nl_max_its = 20
  l_max_its = 50

  # automatic_scaling = true
  # compute_scaling_once = false

  dtmin = 1e-6
  dtmax = 20
  # dt = 1e-3
  [./TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-6
    cutback_factor = .4
    growth_factor = 1.2
    optimal_iterations = 20
    iteration_window = 4
  [../]
[]

[Preconditioning]
  [./SMP]
    type = SMP
    full = true
    solve_type = 'NEWTON'
  [../]
[]

[Postprocessors]
  [./group1_current]
    type = IntegralNewVariablePostprocessor
    variable = group1
    outputs = 'console exodus csv'
  [../]
  [./group2_current]
    type = IntegralNewVariablePostprocessor
    variable = group2
    outputs = 'console exodus csv'
  [../]
  [./group3_current]
    type = IntegralNewVariablePostprocessor
    variable = group3
    outputs = 'console exodus csv'
  [../]
  [./group4_current]
    type = IntegralNewVariablePostprocessor
    variable = group4
    outputs = 'console exodus csv'
  [../]
  [./group5_current]
    type = IntegralNewVariablePostprocessor
    variable = group5
    outputs = 'console exodus csv'
  [../]
  [./group6_current]
    type = IntegralNewVariablePostprocessor
    variable = group6
    outputs = 'console exodus csv'
  [../]
  [./temp_fuel]
    type = ElementAverageValue
    variable = temp
    block = 'fuel'
    outputs = 'exodus console csv'
  [../]
  [./max_temp_fuel]
    type = ElementExtremeValue
    variable = temp
    block = 'fuel'
    value_type = 'max'
    outputs = 'exodus console csv'
  [../]
  [./heat]
    type = ElmIntegTotFissHeatPostprocessor
    outputs = 'csv'
  [../]
  [./coreEndTemp]
    type = SideAverageValue
    variable = temp
    boundary = 'outlet'
    outputs = 'exodus console'
  [../]
  [./inlet_mean_temp]
    type = Receiver
    outputs = 'csv console'
  [../]
  [./coreTempRise]
    type = DifferencePostprocessor
    value1 = 'coreEndTemp'
    value2 = 'inlet_mean_temp'
    outputs = 'csv console'
  [../]

  # Decay heat
  [./outlet_decay_heat1]
    type = SideAverageValue
    variable = heat1
    boundary = 'outlet'
  [../]
  [./inlet_decay_heat1]
    type = Receiver
    execute_on = 'nonlinear'
  [../]
  [./outlet_decay_heat2]
    type = SideAverageValue
    variable = heat2
    boundary = 'outlet'
  [../]
  [./inlet_decay_heat2]
    type = Receiver
    execute_on = 'nonlinear'
  [../]
  [./outlet_decay_heat3]
    type = SideAverageValue
    variable = heat3
    boundary = 'outlet'
  [../]
  [./inlet_decay_heat3]
    type = Receiver
    execute_on = 'nonlinear'
  [../]
[]

[Outputs]
  perf_graph = true
  csv = true
  [./exodus]
    type = Exodus
    execute_on = 'timestep_end'
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]

[MultiApps]
  [./loopApp]
    type = TransientMultiApp
    app_type = MoltresApp
    execute_on = timestep_begin
    positions = '500.0 500.0 0.0'
    input_files = 'sub-all.i'
  [../]
[]

[Transfers]
  [./from_loop]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = loopEndTemp
    to_postprocessor = inlet_mean_temp
    direction = from_multiapp
    reduction_type = maximum
  [../]
  [./to_loop]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = coreEndTemp
    to_postprocessor = coreEndTemp
    direction = to_multiapp
  [../]

  # Decay heat
  [./from_loop_heat1]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = inlet_decay_heat1
    to_postprocessor = inlet_decay_heat1
    direction = from_multiapp
    reduction_type = maximum
  [../]
  [./to_loop_heat1]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = outlet_decay_heat1
    to_postprocessor = outlet_decay_heat1
    direction = to_multiapp
  [../]
  [./from_loop_heat2]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = inlet_decay_heat2
    to_postprocessor = inlet_decay_heat2
    direction = from_multiapp
    reduction_type = maximum
  [../]
  [./to_loop_heat2]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = outlet_decay_heat2
    to_postprocessor = outlet_decay_heat2
    direction = to_multiapp
  [../]
  [./from_loop_heat3]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = inlet_decay_heat3
    to_postprocessor = inlet_decay_heat3
    direction = from_multiapp
    reduction_type = maximum
  [../]
  [./to_loop_heat3]
    type = MultiAppPostprocessorTransfer
    multi_app = loopApp
    from_postprocessor = outlet_decay_heat3
    to_postprocessor = outlet_decay_heat3
    direction = to_multiapp
  [../]

[]

[ICs]
  # [./ux_ic]
  #   type = ConstantIC
  #   variable = ux
  #   value = 0
  # [../]
  # [./uy_ic]
  #   type = ConstantIC
  #   variable = uy
  #   value = 0
  # [../]
  [./temp_ic]
    type = ConstantIC
    variable = temp
    value = ${ini_temp}
  [../]
  [./group1_ic]
    type = ConstantIC
    variable = group1
    value = ${ini_neut}
  [../]
  [./group2_ic]
    type = ConstantIC
    variable = group2
    value = ${ini_neut}
  [../]
  [./group3_ic]
    type = ConstantIC
    variable = group3
    value = ${ini_neut}
  [../]
  [./group4_ic]
    type = ConstantIC
    variable = group4
    value = ${ini_neut}
  [../]
  [./group5_ic]
    type = ConstantIC
    variable = group5
    value = ${ini_neut}
  [../]
  [./group6_ic]
    type = ConstantIC
    variable = group6
    value = ${ini_neut}
  [../]
[]
