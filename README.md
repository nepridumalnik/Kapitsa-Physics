# Kapitsa Physics

Kapitsa Physics — физический движок на Rust для космических симуляторов: орбитального движения, ракет, составных аппаратов и столкновений. Он моделирует твёрдые тела, соединения, контакты и центральную гравитацию. Публичный интерфейс расположен в crate `kapitsa`.

## Подключение

Для проекта внутри workspace:

```toml
[dependencies]
kapitsa = { path = "../kapitsa" }
tokio = { version = "1", features = ["macros", "rt-multi-thread"] }
```

Укажите путь к `crates/kapitsa`, соответствующий расположению вашего проекта. Для примера ниже нужен runtime Tokio.

## Создание и управление симуляцией

```rust
use kapitsa::{Simulation, RigidBody, Transform};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut simulation = Simulation::builder()
        .frequency(120)
        .max_workers(4)
        .spawn_on_tokio(&tokio::runtime::Handle::current())?;

    let client = simulation.client();
    let ground = client.add_body(RigidBody::new_static(Transform::identity())).await?;
    let snapshot = client.snapshot();
    assert!(snapshot.body(ground).is_some());

    simulation.pause()?;
    // Команды клиента продолжают выполняться на паузе.
    simulation.resume()?;
    simulation.stop().await?;
    Ok(())
}
```

`frequency` — базовая частота в Гц, по умолчанию 60. При фиксированном шаге каждый шаг продвигает физическое время на `1 / frequency` секунды. Допустимый диапазон — от 1 до 1 000 000 000 Гц.

`max_workers` — необязательный предел числа вычислительных работников; ноль возвращает ошибку. По умолчанию используется доступный параллелизм системы. Планировщик автоматически меняет активное число работников в пределах пула по измеренному времени шага. Пул не меняет размер при каждом изменении нагрузки.

`time_step_policy` необязателен; по умолчанию используется `TimeStepPolicy::Fixed`. Для адаптивного шага задайте `.time_step_policy(TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()))`.

`pause` останавливает физические шаги, сохраняя текущий множитель времени; `resume` восстанавливает его. На паузе команды применяются и обновлённое состояние публикуется без продвижения времени. `stop().await` останавливает сервис. Клиент можно клонировать и передавать нескольким задачам.

`spawn_on_tokio` использует Tokio для управления и созданный движком Rayon-пул для вычислений. Можно передать существующий Rayon-пул:

```rust
let pool = std::sync::Arc::new(rayon::ThreadPoolBuilder::new().num_threads(4).build()?);
let simulation = Simulation::builder().frequency(120).spawn_on_rayon(pool)?;
```

Для этого способа вызывающему коду не нужен Tokio runtime: движок использует отдельный поток для управления временем. Несколько симуляций могут использовать общий пул, каждая соблюдает собственный предел воркеров. Добавьте зависимость `rayon` в Cargo.toml приложения.

## Время и производительность

`simulation.set_time_multiplier(x)` задаёт скорость симуляционного времени относительно реального: `1.0` — обычная скорость, `0.0` — пауза, больше единицы — ускорение. Отрицательные и нечисловые значения недопустимы. Фактическая скорость зависит от нагрузки.

При перегрузке движок уменьшает скорость продвижения игрового времени по измеренной длительности шага, до 30% от запрошенной скорости. При фиксированном шаге `dt` не меняется. Если шаги не успевают выполняться даже при этом, накапливается отставание. В адаптивном режиме размер `dt` определяется выбранной политикой; измерения и скорость могут колебаться при изменении нагрузки.

Расширенная конфигурация (`Simulation::new_from_config`) доступна, когда параметров builder недостаточно:

| Поле `SimulationConfig` | По умолчанию | Значение |
| --- | --- | --- |
| `tick_rate` | `60` | Базовая частота, Гц |
| `max_catch_up_steps` | `4` | Максимум последовательных шагов за один проход планировщика |
| `time_step_policy` | `Fixed` | Фиксированный шаг или `Adaptive(AdaptiveTimeStepConfig)` |
| `scheduler.min_batch_size` | `128` | Минимальный размер вычислительного пакета |
| `scheduler.max_batches_per_worker` | `4` | Максимальное число пакетов на воркер |
| `workers.max_workers` | `None` | Предел пула; `None` использует доступный параллелизм |
| `workers.scale_up_at` | `0.70` | Доля бюджета шага для пробы увеличения числа воркеров |
| `workers.scale_down_at` | `0.30` | Порог снижения числа воркеров |
| `workers.smoothing` | `0.20` | Вес нового измерения времени шага при сглаживании |
| `central_gravity` | `None` | Совместимый с прежним API одиночный центральный источник |

В `AdaptiveTimeStepConfig` по умолчанию `minimum_step = 10 мкс`, `maximum_step = 500 мс`, `relative_error_tolerance = 0.01`. Эти параметры не гарантируют заданную ошибку для любой сцены.

`simulation.metrics()` сообщает длительность последнего и среднюю длительность шага, бюджет, активные воркеры, пропущенные сроки, накопленное отставание, скорости времени и `load_speed_factor` — замедление из-за нагрузки. Числовые внутренние ограничения runtime находятся в `crates/kapitsa/src/runtime/constants.rs`.

## Тела и состояние

`RigidBody::new_static` создаёт неподвижное тело, `new_kinematic` — кинематическое тело, `new_dynamic` — динамическое тело с `MassProperties`. Положение и ориентацию задаёт `Transform`, линейные величины — `Vec3`. Изменения через клиент используют `BodyId`, который выдаётся движком при добавлении. ID относится к конкретной симуляции.

Пример создания динамического тела со сферическим коллайдером:

```rust
use kapitsa::{Collider, MassProperties, RigidBody, Shape, Transform};
use kapitsa::math::{Mat3, Vec3};
use kapitsa::shape::Sphere;

let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?;
let body = RigidBody::new_dynamic(Transform::identity(), mass)?;
let id = client.add_body(body).await?;
let collider = Collider::new(Shape::Sphere(Sphere::new(1.0)?), Transform::identity());
let collider_id = client.add_collider(id, collider).await?;
client.apply_force(id, Vec3::new(0.0, 1000.0, 0.0)).await?;
```

Масса и тензор инерции здесь приведены для примера; задавайте значения, соответствующие объекту. Клиент умеет изменять положение, скорости, массу и режим CCD; прикладывать силу, момент и импульс; усыплять и пробуждать тело. Для постоянной силы доступны `add_force_source` и `remove_force_source`: точка и сила задаются в локальных координатах тела.

`client.snapshot()` без ожидания возвращает последний опубликованный снимок. `snapshot.body(id)` читает состояние конкретного тела, `snapshot.bodies()` позволяет обойти все тела. `next_snapshot().await` ждёт новую публикацию, но промежуточные снимки могут быть пропущены. Каждая публикация содержит номер шага и время симуляции. На паузе состояние может измениться при неизменных номере шага и времени.

## Коллайдеры и формы

`Collider` прикрепляется к телу через `client.add_collider(body_id, collider)`. Он содержит форму и локальный `Transform`. `CollisionMode::Solid` создаёт физические контакты; `Sensor` сообщает о пересечениях без физического отклика. `CollisionFilter` управляет фильтрацией, материал задаёт свойства контакта. Для динамических тел допускаются только выпуклые формы.

Доступны варианты `Shape`:

| Тип | Назначение |
| --- | --- |
| `Sphere`, `Cuboid` | Сфера и прямоугольный параллелепипед |
| `Capsule`, `Cylinder` | Капсула и цилиндр между двумя точками |
| `TaperedCapsule`, `TaperedCylinder` | Аналогичные формы с разными радиусами концов |
| `Convex` | Выпуклый многогранник |
| `Plane`, `Triangle` | Плоскость и треугольник |
| `TriangleMesh` | Треугольная сетка |
| `HeightField` | Поверхность по сетке высот |
| `LevelSet` | Геометрия, заданная полем расстояний |
| `SkinnedTriangleMesh` | Деформируемая треугольная сетка с костями |

Поддержка конкретного сочетания форм и операций зависит от реализации collision-модуля; ошибки запросов и неподдерживаемой геометрии необходимо обрабатывать. Дополнительные детали — в [docs/collisions.md](docs/collisions.md).

## Соединения и составные объекты

`Joint` связывает два тела. Общие настройки содержат `JointCommon` и локальные `JointFrame`. Варианты:

| Тип | Поведение |
| --- | --- |
| `Fixed` | Фиксирует взаимное положение и ориентацию |
| `Hinge` | Вращение вокруг оси |
| `Slider` | Перемещение вдоль оси |
| `Ball` | Шарнир с вращением в нескольких направлениях |
| `Cylindrical` | Перемещение и вращение вдоль одной оси |
| `Planar` | Движение в плоскости и вращение вокруг её нормали |
| `Distance` | Ограничение расстояния между точками |
| `Spring` | Пружинная связь |
| `ConeTwist` | Шарнир с ограничениями отклонения и скручивания |
| `D6` | Независимая настройка шести степеней свободы |

Клиент умеет добавлять, обновлять, отключать и удалять соединения. `JointLimit`, `JointDrive` и `JointMotion` задают ограничения и приводы там, где они поддерживаются. Разрушение соединения публикуется как событие.

Кластер объединяет детали в одно жёсткое тело. Создайте динамическое тело обычным способом, затем включите кластер и добавьте к нему деталь:

```rust
use kapitsa::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
use kapitsa::joint::JointFrame;
use kapitsa::math::{Mat3, Vec3};
use kapitsa::{MassProperties, RigidBody, Transform};

let part_mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
let body_id = client.add_body(RigidBody::new_dynamic(Transform::identity(), part_mass)?).await?;
let root = client.enable_cluster(body_id).await?;
let weld = WeldDesc {
    frame_a: JointFrame::IDENTITY,
    frame_b: JointFrame::IDENTITY,
    strength: WeldStrength::Unbreakable,
    stiffness: WeldStiffness { linear: 1.0, angular: 1.0 },
};
let part = client.add_cluster_part(root, ClusterPartDesc {
    local_transform: Transform::identity(),
    mass: part_mass,
    colliders: Vec::new(),
    linear_velocity: Vec3::ZERO,
    angular_velocity: Vec3::ZERO,
}, weld).await?;

let cluster = client.cluster(body_id).await?.expect("cluster exists");
let weld_id = cluster.welds()[0];
let bodies = client.break_weld(weld_id).await?;
let detached_body = client.cluster_part(part).await?.expect("part exists").body();
```

В примере начальные скорости и локальные системы координат деталей совпадают; при других положениях и скоростях задайте согласованные системы и поле скоростей. `break_weld` и `detach_part` возвращают ID тел образовавшихся компонент. После разделения снимок уже содержит их состояния. `cluster`, `cluster_part` и `weld` возвращают данные на момент обработки запроса; по ним можно получить актуальные `PartId`, `WeldId`, `BodyId` и коллайдеры.

`add_part_collider` и `remove_part_collider` управляют коллайдерами детали. `destroy_cluster_part` удаляет деталь при допустимой топологии; последнюю деталь кластера удалить этим методом нельзя. `merge_clusters` присоединяет второй кластер к первому. Массу детали меняют через `set_part_mass` с `MassChangeMode::KeepVelocityField` либо через `set_part_mass_with_transfer` с явным `MassTransfer`. Для силы в локальных координатах детали используйте `add_part_force_source`. Соединить тело или деталь с другой целью можно через `add_joint_targets` и `JointTarget`; для обновления такого соединения используйте `update_joint_targets`.

Пределы прочности и жёсткость в `WeldDesc` сейчас сохраняются и проверяются, но автоматическое разрушение сварных соединений по нагрузке не реализовано. Явный `break_weld` выполняет разделение независимо от этих пределов.

## Материалы и гравитация

`PhysicsMaterial` описывает динамическое и статическое трение, упругость и плотность; коллайдер ссылается на материал через `MaterialId`. В примере используется `collider_id` созданного выше коллайдера:

```rust
use kapitsa::PhysicsMaterial;

let material = PhysicsMaterial::new(0.4, 0.5, 0.1, 1000.0)?;
let material_id = client.add_material(material).await?;
client.set_collider_material(collider_id, Some(material_id)).await?;
let current = client.material(material_id).await?;
client.update_material(material_id, PhysicsMaterial::new(0.2, 0.3, 0.0, 1000.0)?).await?;
client.set_collider_material(collider_id, None).await?;
client.remove_material(material_id).await?;
```

Материал, назначенный коллайдеру, удалить нельзя: сначала снимите все назначения. Изменение материала сбрасывает кешированные контакты использующих его коллайдеров. `CollisionConfig` задаёт параметры обработки контактов и CCD и обновляется через клиент.

Источники гравитации добавляются через клиент и идентифицируются `GravityId`. Несколько источников действуют одновременно, их ускорения суммируются:

```rust
use kapitsa::{UniformGravity, math::Vec3};

let gravity = client.add_gravity(UniformGravity::new(Vec3::new(0.0, -9.81, 0.0))?).await?;
client.update_gravity(gravity, UniformGravity::new(Vec3::new(0.0, -3.71, 0.0))?).await?;
client.remove_gravity(gravity).await?;
```

`UniformGravity` — постоянное ускорение во всём пространстве. `CentralGravity` — точечный источник с центром и параметром `μ = GM`, действующий без ограничения расстояния. `SphericalGravity` — та же точечная модель внутри заданного радиуса; вне него ускорение равно нулю. В центре ненулевого точечного источника ускорение не определено и шаг возвращает ошибку. Это не модель распределённой массы внутри планеты. Для совместимости `SimulationConfig::central_gravity` также поддерживает один центральный источник, который суммируется с добавленными полями.

## Запросы, события и пакеты

`client.raycast`, `sweep` и `overlaps` выполняют пространственные запросы. Фильтр `QueryFilter` ограничивает кандидатов. Запросы состояния и пространственные запросы обрабатываются в своей позиции очереди до интегрирования соответствующего шага. Создание и изменение материалов и кластеров также работает на паузе.

`subscribe_collision_events()` и `subscribe_joint_break_events()` создают подписки. Потоки ограничены: медленный получатель получает ошибку `Lagged` с числом пропущенных пакетов.

`client.commands()` создаёт буфер изменений; `try_submit` ставит его в очередь без ожидания и возвращает ID пакета. Очередь ограничена, поэтому при её заполнении пакет возвращается с ошибкой и может быть отправлен повторно. Результаты доступны через `subscribe_batch_results`. Буфер поддерживает перечисленные в `PhysicsCommandBuffer` операции над телами, силами и соединениями; новые операции материалов и кластеров выполняются отдельными командами. Обычные методы клиента ожидают подтверждения применения и публикации состояния.

Если команда уже изменила мир, а последующий физический шаг завершился ошибкой, клиент получает `StepFailedAfterApply`: команда не откатывается, а снимок остаётся на последнем успешном шаге. При создании объекта в такой ситуации его новый ID не возвращается. Результаты запросов только на чтение, выполненных до неудачного шага, сохраняют своё значение.

## Аэродинамика и гидродинамика

Модули `aerodynamics` и `hydrodynamics` сейчас содержат точки расширения без включённой физической модели. Для управления аппаратом пока используйте явные силы и моменты.

## Проверка

```text
cargo test --workspace
cargo clippy -p kapitsa --lib -- -D warnings
```
