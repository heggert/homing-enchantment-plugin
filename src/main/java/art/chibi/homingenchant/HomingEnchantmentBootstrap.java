package art.chibi.homingenchant;

import java.util.logging.Logger;

import org.bukkit.inventory.EquipmentSlotGroup;

import io.papermc.paper.plugin.bootstrap.BootstrapContext;
import io.papermc.paper.plugin.bootstrap.PluginBootstrap;
import io.papermc.paper.registry.RegistryKey;
import io.papermc.paper.registry.data.EnchantmentRegistryEntry;
import io.papermc.paper.registry.event.RegistryEvents;
import io.papermc.paper.registry.keys.EnchantmentKeys;
import io.papermc.paper.registry.keys.ItemTypeKeys;
import io.papermc.paper.registry.set.RegistrySet;
import net.kyori.adventure.key.Key;
import net.kyori.adventure.text.Component;

public class HomingEnchantmentBootstrap implements PluginBootstrap {
    private static final Logger LOGGER = Logger.getLogger("HomingEnchantment");

    @Override
    public void bootstrap(BootstrapContext context) {
        LOGGER.info("HomingEnchantmentBootstrap is initializing...");
        // Register a new handler for the freeze lifecycle event on the enchantment
        // registry
        context.getLifecycleManager().registerEventHandler(RegistryEvents.ENCHANTMENT.freeze().newHandler(event -> {
            event.registry().register(
                    EnchantmentKeys.create(Key.key("chibi:aimbot")),
                    b -> b.description(Component.text("Aimbot"))
                            .supportedItems(RegistrySet.keySet(RegistryKey.ITEM, ItemTypeKeys.CROSSBOW,
                                    ItemTypeKeys.BOW))
                            .anvilCost(1)
                            .maxLevel(25)
                            .weight(10)
                            .minimumCost(EnchantmentRegistryEntry.EnchantmentCost.of(1, 1))
                            .maximumCost(EnchantmentRegistryEntry.EnchantmentCost.of(3, 1))
                            .activeSlots(EquipmentSlotGroup.ANY));
        }));
    }
}