use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};
use std::time::Duration;

use rayon::ThreadPoolBuilder;
use tokio::{
    runtime::Builder,
    sync::{mpsc, oneshot},
    time::timeout,
};

#[test]
fn rayon_workers_run_on_tokio_blocking_pool_with_scoped_work_and_shutdown() {
    const RAYON_WORKERS: usize = 4;
    const BATCHES: usize = 8;
    const VALUES_PER_BATCH: usize = 64;
    const TIMEOUT: Duration = Duration::from_secs(3);

    let runtime = Builder::new_multi_thread()
        .worker_threads(2)
        .max_blocking_threads(1)
        .enable_all()
        .build()
        .expect("host runtime should build");
    let handle = runtime.handle().clone();
    let starts = Arc::new(AtomicUsize::new(0));
    let submitted = Arc::new(AtomicUsize::new(0));
    let (exited_tx, mut exited_rx) = mpsc::unbounded_channel();

    let pool = ThreadPoolBuilder::new()
        .num_threads(RAYON_WORKERS)
        .spawn_handler({
            let starts = Arc::clone(&starts);
            let submitted = Arc::clone(&submitted);
            move |thread| {
                let index = thread.index();
                let starts = Arc::clone(&starts);
                let exited_tx = exited_tx.clone();
                submitted.fetch_add(1, Ordering::Relaxed);
                handle.spawn_blocking(move || {
                    starts.fetch_add(1, Ordering::Release);
                    thread.run();
                    let _ = exited_tx.send(index);
                });
                Ok(())
            }
        })
        .build()
        .expect("Rayon pool should initialize without waiting for the whole blocking pool");

    assert_eq!(pool.current_num_threads(), RAYON_WORKERS);
    assert_eq!(submitted.load(Ordering::Relaxed), RAYON_WORKERS);

    runtime.block_on(async {
        for batch in 0..BATCHES {
            let (result_tx, result_rx) = oneshot::channel();
            pool.spawn(move || {
                let mut values = vec![0_usize; VALUES_PER_BATCH];
                rayon::scope(|scope| {
                    for (index, value) in values.iter_mut().enumerate() {
                        scope.spawn(move |_| *value = index + batch);
                    }
                });
                let _ = result_tx.send(values.into_iter().sum::<usize>());
            });
            let result = timeout(TIMEOUT, result_rx)
                .await
                .expect("Rayon work must not deadlock with one blocking thread")
                .expect("Rayon worker must return its result");
            assert_eq!(
                result,
                (0..VALUES_PER_BATCH).sum::<usize>() + VALUES_PER_BATCH * batch
            );
        }

        assert_eq!(starts.load(Ordering::Acquire), 1);
    });

    drop(pool);
    runtime.block_on(async {
        let mut exited = Vec::with_capacity(RAYON_WORKERS);
        for _ in 0..RAYON_WORKERS {
            exited.push(
                timeout(TIMEOUT, exited_rx.recv())
                    .await
                    .expect("Rayon workers must shut down before the host runtime")
                    .expect("worker exit signal must be sent"),
            );
        }
        exited.sort_unstable();
        assert_eq!(exited, (0..RAYON_WORKERS).collect::<Vec<_>>());
    });
    assert_eq!(starts.load(Ordering::Acquire), RAYON_WORKERS);
    drop(runtime);
}
