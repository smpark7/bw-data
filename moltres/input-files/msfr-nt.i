flow_velocity=0 # cm/s
pre_flow_velocity=0
nt_scale=1e8     # neutron flux scaling factor
pre_scale=1e8    # precursor scaling factor
ini_temp=973     # initial temp
diri_temp=973    # dirichlet BC temp
ini_neut=1

[GlobalParams]
  num_groups = 6
  num_precursor_groups = 8
  temperature = temp
  group_fluxes = 'group1 group2 group3 group4 group5 group6'
  pre_concs = 'pre1 pre2 pre3 pre4 pre5 pre6 pre7 pre8'
  use_exp_form = false
  sss2_input = true
  account_delayed = true
  # alpha = 1
  # eigen = true
[]

[Mesh]
  file = 'msfr-1-16-core-10cm.e'
[../]

[Problem]
  type = FEProblem
  # coord_type = RZ
  # rz_coord_axis = Y
[]

[Variables]
  [./group1]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    # initial_from_file_var = group1
    # initial_from_file_timestep = LATEST
  [../]
  [./group2]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    # initial_from_file_var = group2
    # initial_from_file_timestep = LATEST
  [../]
  [./group3]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    # initial_from_file_var = group3
    # initial_from_file_timestep = LATEST
  [../]
  [./group4]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    # initial_from_file_var = group4
    # initial_from_file_timestep = LATEST
  [../]
  [./group5]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    # initial_from_file_var = group5
    # initial_from_file_timestep = LATEST
  [../]
  [./group6]
    order = FIRST
    family = LAGRANGE
    scaling = ${nt_scale}
    # initial_from_file_var = group6
    # initial_from_file_timestep = LATEST
  [../]
[]

[AuxVariables]
  [./temp]
    order = FIRST
    family = LAGRANGE
    scaling = 1
    initial_condition = 973
  [../]
[]

[Precursors]
  [./pres]
    var_name_base = pre
    block = 'core'
    outlet_boundaries = 'core_top'
    # prec_scale = 1
    constant_velocity_values = true
    u_def = 0
    v_def = ${pre_flow_velocity}
    w_def = 0
    nt_exp_form = false
    family = MONOMIAL
    order = CONSTANT
    transient = false
    inlet_boundaries = 'core_bottom'
    scaling = ${pre_scale}
    # init_from_file = true
    # jac_test = true
  [../]
[]

[Kernels]
  # Neutronics
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
    type = CoupledFissionEigenKernel
    variable = group1
    group_number = 1
  [../]
  [./fission_source_group2]
    type = CoupledFissionEigenKernel
    variable = group2
    group_number = 2
  [../]
  [./fission_source_group3]
    type = CoupledFissionEigenKernel
    variable = group3
    group_number = 3
  [../]
  [./fission_source_group4]
    type = CoupledFissionEigenKernel
    variable = group4
    group_number = 4
  [../]
  [./fission_source_group5]
    type = CoupledFissionEigenKernel
    variable = group5
    group_number = 5
  [../]
  [./fission_source_group6]
    type = CoupledFissionEigenKernel
    variable = group6
    group_number = 6
  [../]

  [./delayed_group1]
    type = DelayedNeutronEigenSource
    variable = group1
    group_number = 1
    block = 'core'
  [../]
  [./delayed_group2]
    type = DelayedNeutronEigenSource
    variable = group2
    group_number = 2
    block = 'core'
  [../]
  [./delayed_group3]
    type = DelayedNeutronEigenSource
    variable = group3
    group_number = 3
    block = 'core'
  [../]
  [./delayed_group4]
    type = DelayedNeutronEigenSource
    variable = group4
    group_number = 4
    block = 'core'
  [../]
  [./delayed_group5]
    type = DelayedNeutronEigenSource
    variable = group5
    group_number = 5
    block = 'core'
  [../]
  [./delayed_group6]
    type = DelayedNeutronEigenSource
    variable = group6
    group_number = 6
    block = 'core'
  [../]
[]

[Materials]
  [./core]
    type = GenericMoltresMaterial
    property_tables_root = '../data/xs-data/group/msfr_full_core_core_'
    interp_type = 'spline'
    prop_names = 'cp'
    prop_values = '1555'
    block = 'core'
  [../]
  [./rho_core]
    type = DerivativeParsedMaterial
    f_name = rho
    function = '(4983.56 - .882 * temp) * .000001'    # kg cm-3
    args = 'temp'
    derivative_order = 1
    block = 'core'
  [../]
  [./k_core]
    type = ParsedMaterial
    f_name = k
    function = '(0.928 + 8.397e-5 * temp) * .01'
    args = 'temp'
    block = 'core'
  [../]
  [./mu_core]
    type = ParsedMaterial
    f_name = mu
    function = 'rho * exp(3689 / temp) * 5.55e-8 * 10000'
    material_property_names = 'rho'
    args = 'temp'
    block = 'core'
  [../]
  [./blanket]
    type = GenericMoltresMaterial
    property_tables_root = '../data/xs-data/group/msfr_full_core_blanket_'
    interp_type = 'spline'
    prop_names = 'cp'
    prop_values = '1555'
    block = 'blanket'
  [../]
  [./rho_blanket]
    type = DerivativeParsedMaterial
    f_name = rho
    function = '(4983.56 - .882 * temp) * .000001'    # kg cm-3
    args = 'temp'
    derivative_order = 1
    block = 'blanket'
  [../]
  [./k_blanket]
    type = ParsedMaterial
    f_name = k
    function = '(0.928 + 8.397e-5 * temp) * .01'
    args = 'temp'
    block = 'blanket'
  [../]
  [./absorb]
    type = GenericMoltresMaterial
    property_tables_root = '../data/xs-data/group/msfr_full_core_absorb_'
    interp_type = 'spline'
    prop_names = 'cp'
    prop_values = '1555'
    block = 'absorb'
  [../]
  [./struc]
    type = GenericMoltresMaterial
    property_tables_root = '../data/xs-data/group/msfr_full_core_struc_'
    interp_type = 'spline'
    prop_names = 'cp'
    prop_values = '1555'
    block = 'bottom_ref outer_ref top_ref'
  [../]
  [./hx]
    type = GenericMoltresMaterial
    property_tables_root = '../data/xs-data/group/msfr_full_core_hx_'
    interp_type = 'spline'
    prop_names = 'cp'
    prop_values = '1555'
    block = 'hx'
  [../]
  [./inlet_outlet]
    type = GenericMoltresMaterial
    property_tables_root = '../data/xs-data/group/msfr_full_core_inlet_'
    interp_type = 'spline'
    prop_names = 'cp'
    prop_values = '1555'
    block = 'inlet outlet'
  [../]
[]

[BCs]
  [./vacuum_group1]
    type = VacuumConcBC
    boundary = 'bottom bottom_outer middle_outer top_outer top'
    variable = group1
  [../]
  [./vacuum_group2]
    type = VacuumConcBC
    boundary = 'bottom bottom_outer middle_outer top_outer top'
    variable = group2
  [../]
  [./vacuum_group3]
    type = VacuumConcBC
    boundary = 'bottom bottom_outer middle_outer top_outer top'
    variable = group3
  [../]
  [./vacuum_group4]
    type = VacuumConcBC
    boundary = 'bottom bottom_outer middle_outer top_outer top'
    variable = group4
  [../]
  [./vacuum_group5]
    type = VacuumConcBC
    boundary = 'bottom bottom_outer middle_outer top_outer top'
    variable = group5
  [../]
  [./vacuum_group6]
    type = VacuumConcBC
    boundary = 'bottom bottom_outer middle_outer top_outer top'
    variable = group6
  [../]
[]

[Executioner]
  type = NonlinearEigen
  free_power_iterations = 4
  source_abs_tol = 1e-12
  source_rel_tol = 1e-8
  output_after_power_iterations = true

  # type = InversePowerMethod
  # max_power_iterations = 50
  # xdiff = 'group1diff'
  
  bx_norm = 'bnorm'
  k0 = 1.0
  pfactor = 1e-2
  l_max_its = 100

  solve_type = 'PJFNK'
  petsc_options = '-snes_converged_reason -ksp_converged_reason -snes_linesearch_monitor'
  # petsc_options_iname = '-pc_type -sub_pc_type'
  # petsc_options_value = 'asm lu'
[]

[Preconditioning]
  [./SMP]
    type = SMP
    full = true
  [../]
[]

[Postprocessors]
  [./bnorm]
    type = ElmIntegTotFissNtsPostprocessor
    group_fluxes = 'group1 group2 group3 group4 group5 group6'
    execute_on = linear
  [../]
  [./group1diff]
    type = ElementL2Diff
    variable = group1
    execute_on = 'linear timestep_end'
    use_displaced_mesh = false
  [../]
  [./group1_flux]
    type = ElementAverageValue
    variable = group1
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./group2_flux]
    type = ElementAverageValue
    variable = group2
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./group3_flux]
    type = ElementAverageValue
    variable = group3
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./group4_flux]
    type = ElementAverageValue
    variable = group4
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./group5_flux]
    type = ElementAverageValue
    variable = group5
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./group6_flux]
    type = ElementAverageValue
    variable = group6
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./temp_fuel]
    type = ElementAverageValue
    variable = temp
    block = 'core'
    outputs = 'exodus console csv'
  [../]
  [./max_temp_fuel]
    type = ElementExtremeValue
    variable = temp
    block = 'core'
    value_type = 'max'
    outputs = 'exodus console csv'
  [../]
  [./temp_blanket]
    type = ElementAverageValue
    variable = temp
    block = 'blanket'
    outputs = 'exodus console csv'
  [../]
  [./heat_fuel]
    type = ElmIntegTotFissHeatPostprocessor
    block = 'core'
    outputs = 'exodus'
  [../]
  [./heat_blanket]
    type = ElmIntegTotFissHeatPostprocessor
    block = 'blanket'
    outputs = 'exodus'
  [../]
  [./heat]
    type = ElmIntegTotFissHeatPostprocessor
    outputs = 'csv'
  [../]
  # [./coreEndTemp]
  #   type = SideAverageValue
  #   variable = temp
  #   boundary = 'fuel_top'
  #   outputs = 'exodus console'
  # [../]
[]

[Outputs]
  perf_graph = true
  csv = true
  [./exodus]
    type = Exodus
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]

[ICs]
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
