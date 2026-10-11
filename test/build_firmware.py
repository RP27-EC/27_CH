"""Compile/link the Keil ARMCC5 target into a temporary directory, without flashing."""
import argparse
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain', type=Path, default=Path('D:/Keil_C51/ARM/ARMCC/bin'))
    args = parser.parse_args()
    project_dir = ROOT / 'MDK-ARM'
    project = ET.parse(project_dir / 'DM-MC02.uvprojx')
    target = project.find('.//Target')
    controls = target.find('TargetOption/TargetArmAds/Cads/VariousControls')
    includes = [(project_dir / p).resolve() for p in controls.findtext('IncludePath').split(';')]
    defines = controls.findtext('Define').split(',') + ['__MICROLIB']
    # RTE adds DSP source aggregates outside the project Groups list.
    dsp = project.find('.//RTE/components/component[@Cgroup="DSP"]/package')
    dsp_dir = args.toolchain.parents[1] / 'PACK/ARM/CMSIS-DSP' / dsp.get('version')
    includes += [dsp_dir / 'Include', dsp_dir / 'PrivateInclude']
    output = Path(tempfile.mkdtemp(prefix='rp27-armcc-'))
    log = []

    def run(command):
        result = subprocess.run([str(x) for x in command], cwd=project_dir,
                                capture_output=True, text=True, errors='replace')
        log.append(result.stdout + result.stderr)
        (output / 'build.log').write_text('\n'.join(log), encoding='utf-8')
        if result.returncode:
            print(result.stdout + result.stderr)
            raise RuntimeError(f'Build failed; log: {output / "build.log"}')

    objects = []
    files = target.findall('Groups/Group/Files/File')
    for source in sorted((dsp_dir / 'Source').glob('*/*.c')):
        # CMSIS-DSP aggregates include the individual functions themselves.
        if source.stem.lower().endswith('functions') or source.stem.lower().endswith('functionsf16') or source.stem.startswith('CommonTables'):
            node = ET.Element('File')
            ET.SubElement(node, 'FilePath').text = str(source)
            ET.SubElement(node, 'FileType').text = '1'
            files.append(node)
    for index, file in enumerate(files):
        if file.findtext('FileOption/CommonProperty/IncludeInBuild') == '0':
            continue
        source = (project_dir / file.findtext('FilePath')).resolve()
        obj = output / f'{index}_{source.stem}.o'
        kind = file.findtext('FileType')
        if kind == '5':  # Project header entries do not generate objects.
            continue
        if kind == '1':
            command = [args.toolchain / 'armcc.exe', '--cpu', 'Cortex-M7.fp.dp',
                       '--c99', '--gnu', '-O1', '--split_sections', '-g', '-c', source, '-o', obj]
            command += [f'-I{p}' for p in includes] + [f'-D{x}' for x in defines]
            run(command)
        elif kind == '2':
            run([args.toolchain / 'armasm.exe', '--cpu', 'Cortex-M7.fp.dp',
                 '--pd', '__MICROLIB SETA 1', '-g', source, '-o', obj])
        else:
            raise RuntimeError(f'Unsupported project file type {kind}: {source}')
        objects.append(obj)
    scatter = (project_dir / target.findtext('TargetOption/TargetArmAds/LDads/ScatterFile')).resolve()
    run([args.toolchain / 'armlink.exe', '--cpu', 'Cortex-M7.fp.dp', '--library_type=microlib',
         '--strict', '--scatter', scatter, '--summary_stderr', '--info', 'summarysizes',
         '--map', '--list', output / 'DM-MC02.map', '-o', output / 'DM-MC02.axf', *objects])
    run([args.toolchain / 'fromelf.exe', '--i32combined', '--output', output / 'DM-MC02.hex',
         output / 'DM-MC02.axf'])
    warnings = sum(item.count('Warning:') for item in log)
    print(f'ARMCC5: {len(objects)} source files compiled and linked; {warnings} warnings.')
    print(log[-2].strip())
    print(f'Output: {output}')


if __name__ == '__main__':
    main()
