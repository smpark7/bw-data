nt_scale=1e-14     # neutron flux scaling factor
pre_scale=1e-11    # precursor scaling factor
ini_temp=953     # initial temp
ini_neut=1e14

[GlobalParams]
  num_groups = 6
  num_precursor_groups = 8
  temperature = temp
  group_fluxes = 'group1 group2 group3 group4 group5 group6'
  pre_concs = 'pre1 pre2 pre3 pre4 pre5 pre6 pre7 pre8'
  use_exp_form = false
  sss2_input = true
  account_delayed = true
  integrate_p_by_parts = true
  gravity = '0 0 0'
  pspg = true
  supg = true
  alpha = .3333
[]

[Mesh]
  file = 'msfr-all-2d-no-decay-heat-temp-800-1300_exodus.e'
  # file = '../msfr-all-2d_exodus.e'
  # file = 'msfr-ins-2d_exodus.e'
[../]

[Problem]
  type = FEProblem
  coord_type = RZ
  rz_coord_axis = Y
[]

[Variables]
  [./group1]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    initial_from_file_var = group1
    initial_from_file_timestep = LATEST
  [../]
  [./group2]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    initial_from_file_var = group2
    initial_from_file_timestep = LATEST
  [../]
  [./group3]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    initial_from_file_var = group3
    initial_from_file_timestep = LATEST
  [../]
  [./group4]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    initial_from_file_var = group4
    initial_from_file_timestep = LATEST
  [../]
  [./group5]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    initial_from_file_var = group5
    initial_from_file_timestep = LATEST
  [../]
  [./group6]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    initial_from_file_var = group6
    initial_from_file_timestep = LATEST
  [../]
  [./temp]
    order = FIRST
    family = LAGRANGE
    scaling = 1e-3
    initial_from_file_var = temp
    initial_from_file_timestep = LATEST
  [../]
  [./ux]
    family = LAGRANGE
    order = FIRST
    block = 'core'
    initial_from_file_var = ux
    initial_from_file_timestep = LATEST
  [../]
  [./uy]
    family = LAGRANGE
    order = FIRST
    block = 'core'
    initial_from_file_var = uy
    initial_from_file_timestep = LATEST
  [../]
  [./p]
    family = LAGRANGE
    order = FIRST
    block = 'core'
    initial_from_file_var = p
    initial_from_file_timestep = LATEST
  [../]
[]

[Precursors]
  [./pres]
    var_name_base = pre
    block = 'core'
    outlet_boundaries = 'core_outlet'
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
    inlet_boundaries = 'core_inlet'
    scaling = ${pre_scale}
    init_from_file = true
  [../]
[]

[Kernels]
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
    block = 'core'
  [../]
  [./delayed_group2]
    type = DelayedNeutronSource
    variable = group2
    group_number = 2
    block = 'core'
  [../]
  [./delayed_group3]
    type = DelayedNeutronSource
    variable = group3
    group_number = 3
    block = 'core'
  [../]
  [./delayed_group4]
    type = DelayedNeutronSource
    variable = group4
    group_number = 4
    block = 'core'
  [../]
  [./delayed_group5]
    type = DelayedNeutronSource
    variable = group5
    group_number = 5
    block = 'core'
  [../]
  [./delayed_group6]
    type = DelayedNeutronSource
    variable = group6
    group_number = 6
    block = 'core'
  [../]

  # Temperature
  [./temp_time_derivative]
    type = INSTemperatureTimeDerivative
    variable = temp
    block = 'core'
  [../]
  [./temp_cond]
    type = MatDiffusion
    variable = temp
    D_name = 'k'
    block = 'blanket absorb hx inlet outlet bottom_ref top_ref outer_ref'
  [../]
  [./temp_source]
    type = TransientFissionHeatSource
    nt_scale=1
    variable = temp
    block = 'core'
  [../]
  [./temp_all]
    type = INSTemperature
    variable = temp
    u = ux
    v = uy
    block = 'core'
  [../]
  
  # Incompressible Navier-Stokes
  [./mass]
    type = INSMassRZ
    variable = p
    u = ux
    v = uy
    p = p
    block = 'core'
  [../]
  [./ux_time_deriv]
    type = INSMomentumTimeDerivative
    variable = ux
    block = 'core'
  [../]
  [./uy_time_deriv]
    type = INSMomentumTimeDerivative
    variable = uy
    block = 'core'
  [../]
  [./x_momentum_space]
    type = INSMomentumTractionFormRZ
    variable = ux
    u = ux
    v = uy
    p = p
    component = 0
    block = 'core'
  [../]
  [./y_momentum_space]
    type = INSMomentumTractionFormRZ
    variable = uy
    u = ux
    v = uy
    p = p
    component = 1
    block = 'core'
  [../]
[]

[DGKernels]
  [./diff_pre1]
    type = DGDiffusion
    variable = pre1
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre2]
    type = DGDiffusion
    variable = pre2
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre3]
    type = DGDiffusion
    variable = pre3
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre4]
    type = DGDiffusion
    variable = pre4
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre5]
    type = DGDiffusion
    variable = pre5
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre6]
    type = DGDiffusion
    variable = pre6
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre7]
    type = DGDiffusion
    variable = pre7
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
  [./diff_pre8]
    type = DGDiffusion
    variable = pre8
    diff = 114 # 4e-1 / .85 / rho
    epsilon = -1
    sigma = 6
    block = 'core'
  [../]
[]

[Materials]
  [./core]
    type = GenericMoltresMaterial
    property_tables_root = '../../data/xs-data-temp/group/msfr_full_core_core_'
    interp_type = 'spline'
    prop_names = 'cp rho k mu'
    prop_values = '1555 4.12487e-3 731.77 4.001e-1'
    block = 'core'
  [../]
  [./blanket]
    type = GenericMoltresMaterial
    property_tables_root = '../../data/xs-data-temp/group/msfr_full_core_blanket_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '1555 4.12487e-3 1.0097e-2'
    block = 'blanket'
  [../]
  [./absorb]
    type = GenericMoltresMaterial
    property_tables_root = '../../data/xs-data-temp/group/msfr_full_core_absorb_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '1064 2.52e-3 .295'
    block = 'absorb'
  [../]
  [./struc]
    type = GenericMoltresMaterial
    property_tables_root = '../../data/xs-data-temp/group/msfr_full_core_struc_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '427 1e-2 .236'
    block = 'bottom_ref outer_ref top_ref'
  [../]
  [./hx]
    type = GenericMoltresMaterial
    property_tables_root = '../../data/xs-data-temp/group/msfr_full_core_hx_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '1555 4.12487e-3 1.0097e-2'
    block = 'hx'
  [../]
  [./inlet_outlet]
    type = GenericMoltresMaterial
    property_tables_root = '../../data/xs-data-temp/group/msfr_full_core_inlet_'
    interp_type = 'spline'
    prop_names = 'cp rho k'
    prop_values = '1555 4.12487e-3 1.0097e-2'
    block = 'inlet outlet'
  [../]
[]

[Functions]
  [./velFunc]
    type = ParsedFunction
    value = '-508.5025 * 4 * ((y - 100) / 18.75 - ((y - 100) / 18.75)^2) * (1)'
  [../]
[]

[BCs]
  [./temp_core_inlet]
    boundary = 'core_inlet'
    type = PostprocessorDirichletBC
    postprocessor = inlet_mean_temp
    variable = temp
  [../]
  [./vacuum_group1]
    type = VacuumConcBC
    boundary = 'bottom outer top'
    variable = group1
  [../]
  [./vacuum_group2]
    type = VacuumConcBC
    boundary = 'bottom outer top'
    variable = group2
  [../]
  [./vacuum_group3]
    type = VacuumConcBC
    boundary = 'bottom outer top'
    variable = group3
  [../]
  [./vacuum_group4]
    type = VacuumConcBC
    boundary = 'bottom outer top'
    variable = group4
  [../]
  [./vacuum_group5]
    type = VacuumConcBC
    boundary = 'bottom outer top'
    variable = group5
  [../]
  [./vacuum_group6]
    type = VacuumConcBC
    boundary = 'bottom outer top'
    variable = group6
  [../]
  [./ux_dirichlet]
    type = DirichletBC
    boundary = 'core_outer core_bottom core_top core_axial'
    variable = ux
    value = 0
  [../]
  [./uy_dirichlet]
    type = DirichletBC
    boundary = 'core_outer core_bottom core_top core_inlet'
    variable = uy
    value = 0
  [../]
  [./ux_inlet]
    type = FunctionDirichletBC
    boundary = 'core_inlet'
    variable = ux
    function = 'velFunc'
  [../]
  [./pre1_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre1
    uvel = ux
    vvel = uy
  [../]
  [./pre2_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre2
    uvel = ux
    vvel = uy
  [../]
  [./pre3_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre3
    uvel = ux
    vvel = uy
  [../]
  [./pre4_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre4
    uvel = ux
    vvel = uy
  [../]
  [./pre5_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre5
    uvel = ux
    vvel = uy
  [../]
  [./pre6_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre6
    uvel = ux
    vvel = uy
  [../]
  [./pre7_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre7
    uvel = ux
    vvel = uy
  [../]
  [./pre8_outlet]
    type = CoupledOutflowBC
    boundary = 'core_outlet'
    variable = pre8
    uvel = ux
    vvel = uy
  [../]
[]

[Executioner]
  type = Transient
  end_time = 2000

  nl_rel_tol = 1e-8
  nl_abs_tol = 1e-1

  solve_type = 'NEWTON'
  petsc_options = '-snes_converged_reason -ksp_converged_reason -snes_linesearch_monitor'
  petsc_options_iname = '-pc_type -pc_factor_shift_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu       NONZERO               superlu_dist'
  line_search = 'none'

  nl_max_its = 20
  l_max_its = 50

  dtmin = 1e-6
  dtmax = 1e-1
  [./TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-6 # 1e-4
    cutback_factor = .5
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
  [./temp_core]
    type = ElementAverageValue
    variable = temp
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./max_temp_core]
    type = ElementExtremeValue
    variable = temp
    block = 'core'
    value_type = 'max'
    outputs = 'exodus console csv'
  [../]
  [./heat_core]
    type = ElmIntegTotFissHeatPostprocessor
    block = 'core'
    outputs = 'exodus'
  [../]
  [./heat]
    type = ElmIntegTotFissHeatPostprocessor
    outputs = 'csv'
  [../]
  [./coreEndTemp]
    type = SideAverageValue
    variable = temp
    boundary = 'core_outlet'
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
[]

[Outputs]
  perf_graph = true
  csv = true
  [./exodus]
    type = Exodus
    execute_on = 'timestep_end'
  [../]
  [./my_checkpoint]
    type = Checkpoint
    num_files = 2
    interval = 3
    end_time = 2000
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
    input_files = 'sub-all-2d-no-decay-heat-temp-800-1300.i'
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
[]

