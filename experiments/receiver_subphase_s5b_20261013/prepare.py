"""Mechanical timing-only overlays. Never write to original source/runtime trees."""
import difflib
import hashlib
import json
from pathlib import Path
import re
import shutil

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
ROOT = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
OLD = Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate')
CACHE = Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/build')
CAPTURE = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared')
BUILD = ROOT / 'build'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()


def once(text, old, new):
    assert text.count(old) == 1, (old[:100], text.count(old))
    return text.replace(old, new)


def calls(text, pattern, tag, expected=None):
    """Wrap balanced call expressions, without changing their argument order."""
    matches = list(re.finditer(pattern + r'\(', text))
    if expected is not None: assert len(matches) == expected, (pattern, len(matches), expected)
    for m in reversed(matches):
        depth = 1; end = m.end()
        while depth:
            if text[end] == '(': depth += 1
            elif text[end] == ')': depth -= 1
            end += 1
        expression = text[m.start():end]
        text = text[:m.start()] + 's5b::Timed(s5b::' + tag + ', [&] { return ' + expression + '; })' + text[end:]
    return text


def main():
    ROOT.mkdir(exist_ok=False); BUILD.mkdir()
    protected = json.loads((CAPTURE / 'protected_manifest.json').read_text())
    originals = {
      'DiagnosticReplay.cpp': OLD / 'DiagnosticReplay.cpp',
      'SourceWnoaModel.h': OLD / 'SourceWnoaModel.h',
      'ExactLinearizationMemo.h': OLD / 'ExactLinearizationMemo.h',
      'FullPublicationRefinement.h': CACHE / 'FullPublicationRefinement.h',
      'CachedGaussianEvaluation.h': CACHE / 'CachedGaussianEvaluation.h',
      'offline_process/BoundedISAM2Refinement.h': REPO / 'online_fgo/include/offline_process/BoundedISAM2Refinement.h',
      'offline_process/TransactionalISAM2Rebuild.h': REPO / 'online_fgo/include/offline_process/TransactionalISAM2Rebuild.h',
      'factor/motion/GPWNOAPrior.h': REPO / 'online_fgo/include/factor/motion/GPWNOAPrior.h',
    }
    for path in [*originals.values(), OLD / 'replay', CAPTURE / 'capacity160_on_receiver/events.csv']:
        actual = sha(path)
        if str(path) in protected: assert actual == protected[str(path)], str(path)
        protected[str(path)] = actual
    assert protected[str(OLD / 'replay')] == '9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
    with (ROOT / 'protected.json').open('x') as f: json.dump(protected, f, indent=2)
    changes = []
    for name, path in originals.items():
        original = path.read_text(); text = '#include "ReceiverTiming.h"\n' + original
        if name == 'DiagnosticReplay.cpp':
            text = once(text, 'const auto result=numerical_audit::Refine(isam2_);',
              's5b::ContextScope publication_context(s5b::Publishing);\n    const auto result=s5b::Timed(s5b::Publication, [&] { return numerical_audit::Refine(isam2_); });')
            text = once(text, 'e2_memo::Flush(out,index,physical,phase);',
              's5b::Timed(s5b::MemoIO, [&] { e2_memo::Flush(out,index,physical,phase); });')
            text = once(text, '    const auto path=std::filesystem::path(out)/"publication_convergence.csv";',
              '    { s5b::Scope publication_io(s5b::PublicationIO);\n    const auto path=std::filesystem::path(out)/"publication_convergence.csv";')
            text = once(text, "<<s.consensus<<','<<s.qr_disagreement<<','<<s.local_prediction<<'\\n';\n  }",
              "<<s.consensus<<','<<s.qr_disagreement<<','<<s.local_prediction<<'\\n';\n    }\n    s5b::Flush(index,physical,phase);\n  }")
            text = once(text, '    if (!isam2_ || graph.size() != 1) {',
              '    s5b::ContextScope revision_context(s5b::Replacing);\n    s5b::Scope revision_timer(s5b::Revision);\n    if (!isam2_ || graph.size() != 1) {')
            text = once(text, '      isam2_->update(graph, values);',
              '      s5b::ContextScope support_context(s5b::Supporting);\n      s5b::Timed(s5b::SupportUpdate, [&] { isam2_->update(graph, values); });')
            text = calls(text, r'isam2_->update', 'IsamUpdate', 2)
            text = once(text, 'return isam2_ ? isam2_->calculateEstimate() : fixed_lag_->calculateEstimate();',
              'return s5b::Timed(s5b::Estimate, [&] { return isam2_ ? isam2_->calculateEstimate() : fixed_lag_->calculateEstimate(); });')
            text = calls(text, r'event_stream->Next', 'EventRead')
            text = once(text, '    return Run(argc, argv);',
              '    s5b::Finalizer finish;\n    return s5b::Timed(s5b::Run, [&] { return Run(argc, argv); });')
        elif name == 'FullPublicationRefinement.h':
            text = calls(text, r'std::make_unique<gtsam::ISAM2>', 'SolverCopy', 2)
            text = calls(text, r'(?:candidate|trial)->update', 'IsamUpdate', 2)
            text = calls(text, r'(?:solver|candidate|rebuilt|trial|fresh)->calculate(?:Best)?Estimate', 'Estimate')
            text = calls(text, r'(?:solver|candidate)->getFactorsUnsafe\(\)\.error', 'GraphError', 2)
            text = calls(text, r'before\.error', 'GraphError', 3)
            text = calls(text, r'fgo::offline::GaussianCostChange', 'StableCost', 2)
            text = calls(text, r'fgo::offline::RebuildFullHistoryQR', 'Recovery', 2)
            text = calls(text, r'publication_cache::CumulativeChange', 'StableCost', 2)
            text = once(text, 'for(size_t pass=iteration+1;pass<=limit;++pass) {',
              'for(size_t pass=iteration+1;pass<=limit;++pass) {\n              s5b::Scope polish_timer(s5b::Polish);')
        elif name == 'CachedGaussianEvaluation.h':
            text = calls(text, r'graph_\.linearize', 'GraphLinearize', 1)
            text = once(text, '  double gradient() const {', '  double gradient() const {\n    s5b::Scope gradient_timer(s5b::Gradient);')
            # Materialize linear() in original evaluation order within each call.
            text = calls(text, r'linear\(\)\.gradientAtZero', 'GradientVector', 1)
            text = calls(text, r'linear\(\)\.hessianDiagonal', 'HessianDiagonal', 1)
        elif name == 'offline_process/BoundedISAM2Refinement.h':
            text = once(text, 'const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {',
              'const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {\n  s5b::Scope gradient_timer(s5b::Gradient);')
            text = calls(text, r'graph\.linearize', 'GraphLinearize', 1)
            text = calls(text, r'linear->gradientAtZero', 'GradientVector', 1)
            text = calls(text, r'linear->hessianDiagonal', 'HessianDiagonal', 1)
        elif name == 'offline_process/TransactionalISAM2Rebuild.h':
            text = once(text, 'const auto result=update(graph,seed);',
              'const auto result=s5b::Timed(s5b::IsamUpdate, [&] { return update(graph,seed); });')
        elif name == 'ExactLinearizationMemo.h':
            text = once(text, '    if(!enabled_) return Base::linearize(values);',
              '    s5b::Scope memo_timer(Kind == 0 ? s5b::MotionLinearize : s5b::SourceLinearize);\n    if(!enabled_) return Base::linearize(values);')
            text = calls(text, r'e2_probe::Signature', 'Signature', 1)
            text = calls(text, r'cached_->clone', 'HitClone', 1)
            text = once(text, 'auto result=Base::linearize(values);',
              'auto result=s5b::Timed(Kind == 0 ? s5b::MotionBase : s5b::SourceBase, [&] { return Base::linearize(values); });')
        elif name == 'SourceWnoaModel.h':
            text = calls(text, r'(?<!\w)interpolator->interpolatePose', 'NumericInterpolation', 2)
            text = calls(text, r'twist_interpolator->interpolatePose', 'AnalyticInterpolation', 1)
            text = once(text, '    std::vector<gtsam::Matrix> hi, hj;',
              '    s5b::Scope source_timer(s5b::SourceError);\n    std::vector<gtsam::Matrix> hi, hj;')
        elif name == 'factor/motion/GPWNOAPrior.h':
            text = once(text, '#include "GPPriorBase.h"', '#include "factor/motion/GPPriorBase.h"')
            text = once(text, '      using namespace gtsam;',
              '      s5b::Scope motion_timer(s5b::MotionError);\n      using namespace gtsam;')
            text = calls(text, r'gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>', 'MotionPoseDerivative', 2)
            text = calls(text, r'gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>', 'MotionTwistDerivative', 4)
        target = BUILD / name; target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('x') as f: f.write(text)
        changes.extend(difflib.unified_diff(original.splitlines(True), text.splitlines(True), str(path), str(target)))
    for name in ['ReceiverTiming.h', 'test_timing.cpp']:
        shutil.copy2(HERE / name, BUILD / name)
    shutil.copy2(OLD / 'SnapshotAudit.h', BUILD / 'SnapshotAudit.h')
    # Quoted includes in existing tests must resolve to the diagnostic overlay.
    for name in ['test_e2_audit.cpp', 'test_twist.cpp', 'test_memo.cpp']:
        shutil.copy2(OLD / name, BUILD / name)
    shutil.copy2(OLD.parent.parent / 'gnssfgo_vw_jacobian_20261011/build/test_cache.cpp', BUILD / 'test_cache.cpp')
    shutil.copy2(CACHE / 'test_refinement.cpp', BUILD / 'test_refinement.cpp')
    with (ROOT / 'instrumentation.diff').open('x') as f: f.writelines(changes)
    # Exact prefix, no reordered rows, followed by explicit stream end.
    journal = CAPTURE / 'capacity160_on_receiver/events.csv'
    first = None; n = 0; last = None
    with journal.open() as src, (ROOT / 'prefix5.csv').open('x') as dst:
        for line in src:
            cols = line.split(',')
            if cols[0] == 'E': break
            timestamp = int(cols[1])
            if first is None: first = timestamp
            if timestamp > first + 5000000000: break
            dst.write(line); n += 1; last = timestamp
        dst.write(f'E,{last}\n')
    with (ROOT / 'preparation.json').open('x') as f:
        json.dump(dict(originals={str(p): sha(p) for p in originals.values()},
          overlay={str(p.relative_to(BUILD)):sha(p) for p in BUILD.rglob('*') if p.is_file()},
          prefix_rows=n, prefix_sha256=sha(ROOT/'prefix5.csv'), journal_sha256=sha(journal)), f, indent=2)
    print('Prepared timing-only overlay', len(originals), 'files;', n, 'prefix events')


if __name__ == '__main__': main()
